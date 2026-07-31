/**
 * @file imu.c
 * @brief ICM-20948 IMU module implementation.
 */

#include "imu.h"

#include "console.h"
#include "i2c.h"
#include "impact_detector.h"
#include "time_capture.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>

/** ICM-20948 I2C address when the AD0 pin is low. */
#define IMU_I2C_ADDRESS_LOW (0x68U << 1U)

/** ICM-20948 I2C address when the AD0 pin is high. */
#define IMU_I2C_ADDRESS_HIGH (0x69U << 1U)

/** Expected value of the ICM-20948 WHO_AM_I register. */
#define IMU_WHO_AM_I_VALUE 0xEAU

/** Timeout used for blocking I2C operations. */
#define IMU_I2C_TIMEOUT_MS 100U

/** Delay before another I2C recovery attempt. */
#define IMU_RECOVERY_DELAY_MS 500U

/** Number of consecutive measurement bytes read from the ICM-20948. */
#define IMU_MEASUREMENT_SIZE 14U

/** Gyroscope sample-rate divider for 100 Hz output. */
#define IMU_GYROSCOPE_SAMPLE_RATE_DIVIDER 10U

/** Accelerometer sample-rate divider for approximately 102 Hz output. */
#define IMU_ACCELEROMETER_SAMPLE_RATE_DIVIDER 10U

/** Positive half of the signed 16-bit accelerometer output range. */
#define IMU_ACCELEROMETER_OUTPUT_RANGE 32768.0f

/** Mask of the ACCEL_FS_SEL field in ACCEL_CONFIG. */
#define IMU_ACCELEROMETER_RANGE_MASK 0x06U

/** Bit position of the ACCEL_FS_SEL field in ACCEL_CONFIG. */
#define IMU_ACCELEROMETER_RANGE_POSITION 1U

/**
 * @brief ICM-20948 register addresses used by this module.
 */
typedef enum
{
    IMU_REGISTER_WHO_AM_I = 0x00U,
    IMU_REGISTER_PWR_MGMT_1 = 0x06U,
    IMU_REGISTER_PWR_MGMT_2 = 0x07U,
    IMU_REGISTER_INT_ENABLE_1 = 0x11U,
    IMU_REGISTER_ACCEL_XOUT_H = 0x2DU,
    IMU_REGISTER_REG_BANK_SEL = 0x7FU,
    IMU_REGISTER_GYRO_SMPLRT_DIV = 0x00U,
    IMU_REGISTER_ACCEL_SMPLRT_DIV_1 = 0x10U,
    IMU_REGISTER_ACCEL_SMPLRT_DIV_2 = 0x11U,
    IMU_REGISTER_ACCEL_CONFIG = 0x14U
} IMU_Register;

/**
 * @brief ICM-20948 register-bank selection values.
 */
typedef enum
{
    IMU_REGISTER_BANK_0 = 0x00U,
    IMU_REGISTER_BANK_2 = 0x20U
} IMU_RegisterBank;

/** Handle of the task responsible for IMU operation. */
static TaskHandle_t imu_task_handle;

/** Detected ICM-20948 HAL I2C address. */
static uint16_t imu_device_address;

/** Accelerometer full-scale range read from ACCEL_CONFIG, in g. */
static uint8_t imu_accelerometer_full_scale_g;

/**
 * @brief Combine two register bytes into one signed 16-bit value.
 *
 * @param high_byte Most significant byte.
 * @param low_byte Least significant byte.
 * @return Combined signed measurement value.
 */
static int16_t IMU_CombineBytes(uint8_t high_byte, uint8_t low_byte)
{
    return (int16_t)(((uint16_t)high_byte << 8U) | low_byte);
}

/**
 * @brief Calculate the total acceleration magnitude.
 *
 * The accelerometer reports separate signed values for the X, Y and Z axes.
 * Their vector magnitude represents the total measured acceleration without
 * depending on how the sensor axes are oriented relative to the vehicle. At
 * rest, the magnitude is approximately 1 g because of gravity; an impact
 * produces a deviation from the slowly changing baseline used by the impact
 * detector.
 *
 * @param data Pointer to a raw IMU measurement.
 * @return Total acceleration magnitude in g.
 */
static float IMU_CalculateAccelerationMagnitude(const IMU_Data *data)
{
    float magnitude;

    /* Calculate the Euclidean magnitude sqrt(x^2 + y^2 + z^2) in raw units. */
    magnitude = sqrtf((float)data->accelerometer_x * data->accelerometer_x +
                      (float)data->accelerometer_y * data->accelerometer_y +
                      (float)data->accelerometer_z * data->accelerometer_z);

    /*
     * Convert the raw magnitude to g using the active full-scale range.
     * Each axis is a signed 16-bit value from -32768 to +32767, so 2^15 =
     * 32768 raw counts represent the selected positive full-scale value.
     *
     * For example, in the +/-2 g range the sensitivity is
     * 32768 / 2 = 16384 counts/g. A raw magnitude of 16384 therefore gives:
     *
     *     16384 * 2 g / 32768 = 1 g
     *
     * The same formula automatically applies to the +/-4 g, +/-8 g and
     * +/-16 g ranges by using the full-scale value read from ACCEL_CONFIG.
     */
    return magnitude * imu_accelerometer_full_scale_g / IMU_ACCELEROMETER_OUTPUT_RANGE;
}

/**
 * @brief Read one byte from an ICM-20948 register.
 *
 * @param register_address Register address in the currently selected bank.
 * @param value Pointer to the destination byte.
 * @return true if the register was read successfully; otherwise false.
 */
static bool IMU_ReadRegister(IMU_Register register_address, uint8_t *value)
{
    if (value == NULL)
    {
        return false;
    }

    return (HAL_I2C_Mem_Read(&hi2c1,
                             imu_device_address,
                             (uint16_t)register_address,
                             I2C_MEMADD_SIZE_8BIT,
                             value,
                             1U,
                             IMU_I2C_TIMEOUT_MS) == HAL_OK);
}

/**
 * @brief Write one byte to an ICM-20948 register.
 *
 * @param register_address Register address in the currently selected bank.
 * @param value Value to write.
 * @return true if the register was written successfully; otherwise false.
 */
static bool IMU_WriteRegister(IMU_Register register_address, uint8_t value)
{
    return (HAL_I2C_Mem_Write(&hi2c1,
                              imu_device_address,
                              (uint16_t)register_address,
                              I2C_MEMADD_SIZE_8BIT,
                              &value,
                              1U,
                              IMU_I2C_TIMEOUT_MS) == HAL_OK);
}

/**
 * @brief Select an ICM-20948 register bank.
 *
 * @param bank Register bank to select.
 * @return true if the bank was selected successfully; otherwise false.
 */
static bool IMU_SelectRegisterBank(IMU_RegisterBank bank)
{
    return IMU_WriteRegister(IMU_REGISTER_REG_BANK_SEL, (uint8_t)bank);
}

/**
 * @brief Configure the ICM-20948 for interrupt-driven measurements.
 *
 * The accelerometer and gyroscope are enabled with an output rate of
 * approximately 100 Hz. A rising edge is generated on INT when new raw sensor
 * data are ready.
 *
 * ICM-20948 datasheet:
 * https://invensense.tdk.com/wp-content/uploads/2024/03/DS-000189-ICM-20948-v1.6.pdf
 *
 * @return true if all configuration registers were written successfully;
 * otherwise false.
 */
static bool IMU_Configure(void)
{
    uint8_t accelerometer_config;
    uint8_t accelerometer_range_selection;

    /*
     * PWR_MGMT_1, PWR_MGMT_2 and INT_ENABLE_1 are located in user bank 0.
     * See REG_BANK_SEL, Section 8.65, page 54.
     */
    if (!IMU_SelectRegisterBank(IMU_REGISTER_BANK_0))
    {
        return false;
    }

    /*
     * Wake the device and select its best available clock source (CLKSEL = 1).
     * All other fields remain clear: low-power mode is disabled and the
     * temperature sensor stays enabled.
     * See PWR_MGMT_1, Section 8.4, page 37.
     */
    if (!IMU_WriteRegister(IMU_REGISTER_PWR_MGMT_1, 0x01U))
    {
        return false;
    }

    /*
     * Enable all three accelerometer axes and all three gyroscope axes
     * (DISABLE_ACCEL = 000, DISABLE_GYRO = 000).
     * See PWR_MGMT_2, Section 8.5, page 38.
     */
    if (!IMU_WriteRegister(IMU_REGISTER_PWR_MGMT_2, 0x00U))
    {
        return false;
    }

    /*
     * The gyroscope and accelerometer sample-rate dividers and ACCEL_CONFIG
     * are located in user bank 2.
     */
    if (!IMU_SelectRegisterBank(IMU_REGISTER_BANK_2))
    {
        return false;
    }

    /*
     * Set the gyroscope divider to 10. With the 1.125 kHz internal rate,
     * ODR = 1125 / (1 + 10) = approximately 102.3 Hz.
     * See GYRO_SMPLRT_DIV, Section 10.1, page 59.
     */
    if (!IMU_WriteRegister(IMU_REGISTER_GYRO_SMPLRT_DIV, IMU_GYROSCOPE_SAMPLE_RATE_DIVIDER))
    {
        return false;
    }

    /*
     * Clear the upper four bits of the 12-bit accelerometer sample-rate
     * divider. ACCEL_SMPLRT_DIV_1 contains divider bits [11:8].
     * See ACCEL_SMPLRT_DIV_1, Section 10.11, page 63.
     */
    if (!IMU_WriteRegister(IMU_REGISTER_ACCEL_SMPLRT_DIV_1, 0x00U))
    {
        return false;
    }

    /*
     * Set divider bits [7:0] to 10. Together with the upper value above,
     * ODR = 1125 / (1 + 10) = approximately 102.3 Hz.
     * See ACCEL_SMPLRT_DIV_2, Section 10.12, page 63.
     */
    if (!IMU_WriteRegister(IMU_REGISTER_ACCEL_SMPLRT_DIV_2, IMU_ACCELEROMETER_SAMPLE_RATE_DIVIDER))
    {
        return false;
    }

    /*
     * Read ACCEL_FS_SEL[1:0] from ACCEL_CONFIG to determine the active
     * full-scale range: 00 = +/-2 g, 01 = +/-4 g, 10 = +/-8 g,
     * 11 = +/-16 g. This function does not change the configured range.
     * See ACCEL_CONFIG, Section 10.15, page 64.
     */
    if (!IMU_ReadRegister(IMU_REGISTER_ACCEL_CONFIG, &accelerometer_config))
    {
        return false;
    }

    /*
     * ACCEL_FS_SEL occupies bits [2:1] of ACCEL_CONFIG. Mask all other bits
     * with 0x06 and shift the field right by one position to obtain its
     * two-bit numeric value:
     *
     *     ACCEL_FS_SEL = 00 -> selection 0 -> +/-2 g
     *     ACCEL_FS_SEL = 01 -> selection 1 -> +/-4 g
     *     ACCEL_FS_SEL = 10 -> selection 2 -> +/-8 g
     *     ACCEL_FS_SEL = 11 -> selection 3 -> +/-16 g
     *
     * Starting with 2 and shifting it left by the selection value produces
     * the positive full-scale value used for conversion: 2, 4, 8 or 16 g.
     * See ACCEL_CONFIG, Section 10.15, page 64.
     */
    accelerometer_range_selection = (accelerometer_config & IMU_ACCELEROMETER_RANGE_MASK) >> IMU_ACCELEROMETER_RANGE_POSITION;
    imu_accelerometer_full_scale_g = (uint8_t)(2U << accelerometer_range_selection);

    /* Return to user bank 0 before configuring the interrupt source. */
    if (!IMU_SelectRegisterBank(IMU_REGISTER_BANK_0))
    {
        return false;
    }

    /*
     * Enable RAW_DATA_0_RDY_EN so new raw sensor data generate an interrupt
     * on INT1. INT_PIN_CFG is left unchanged; its reset value configures an
     * active-high, push-pull, 50 us pulse.
     * See INT_ENABLE_1, Section 8.8, page 39, and INT_PIN_CFG,
     * Section 8.6, page 38.
     */
    return IMU_WriteRegister(IMU_REGISTER_INT_ENABLE_1, 0x01U);
}

/**
 * @brief Recover communication with the ICM-20948 after an I2C error.
 *
 * Resets the STM32 I2C peripheral, detects the sensor again and restores its
 * configuration.
 *
 * @return true if communication and sensor configuration were restored;
 * otherwise false.
 */
static bool IMU_Recover(void)
{
    (void)HAL_I2C_DeInit(&hi2c1);
    vTaskDelay(pdMS_TO_TICKS(10U));
    MX_I2C1_Init();

    if (!IMU_Test())
    {
        return false;
    }

    return IMU_Configure();
}

/**
 * @brief Run the IMU task.
 *
 * Verifies that the ICM-20948 is available. Sensor configuration and
 * measurement reading will be added here in the next development step.
 *
 * @param argument Unused task argument.
 */
static void IMU_Task(void *argument)
{
    IMU_Data data;

    (void)argument;

    if (!IMU_Test())
    {
        vTaskDelete(NULL);
        return;
    }

    if (!IMU_Configure())
    {
        Console_Printf("IMU: configuration failed\r\n");
        vTaskDelete(NULL);
        return;
    }

    Console_Printf("IMU: measurement interrupt enabled\r\n");
    Console_Printf("IMU: accelerometer range=+/-%u g\r\n", imu_accelerometer_full_scale_g);

    for (;;)
    {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (IMU_Read(&data))
        {
            ImpactDetector_ProcessSample(&data);
        }
        else
        {
            Console_Printf("IMU: measurement read failed\r\n");

            while (!IMU_Recover())
            {
                Console_Printf("IMU: recovery failed, retrying\r\n");
                vTaskDelay(pdMS_TO_TICKS(IMU_RECOVERY_DELAY_MS));
            }

            Console_Printf("IMU: communication recovered\r\n");
        }
    }
}

/**
 * @brief Create the task responsible for IMU operation.
 *
 * @return true if the IMU task was created successfully; otherwise false.
 */
bool IMU_Init(void)
{
    return (xTaskCreate(IMU_Task,
                        "IMU",
                        512U,
                        NULL,
                        tskIDLE_PRIORITY + 1U,
                        &imu_task_handle) == pdPASS);
}

/**
 * @brief Check the ICM-20948 I2C address and WHO_AM_I register.
 *
 * ICM-20948 datasheet:
 * https://invensense.tdk.com/wp-content/uploads/2024/03/DS-000189-ICM-20948-v1.6.pdf
 *
 * @return true if an ICM-20948 is detected; otherwise false.
 */
bool IMU_Test(void)
{
    uint8_t who_am_i;

    /*
     * The ICM-20948 has one of two possible 7-bit I2C addresses. Pin AD0
     * supplies the least significant address bit: AD0 low selects 0x68 and
     * AD0 high selects 0x69. The firmware cannot read AD0 before it knows the
     * device address, so it does not try to determine the pin level directly.
     * Instead, it probes both valid addresses and stores the one that responds.
     * The address constants are shifted left by one bit because STM32 HAL
     * expects the 7-bit address in bits [7:1].
     * See I2C Interface, Section 6.2, page 28.
     */
    if (HAL_I2C_IsDeviceReady(&hi2c1, IMU_I2C_ADDRESS_LOW, 2U, IMU_I2C_TIMEOUT_MS) == HAL_OK)
    {
        imu_device_address = IMU_I2C_ADDRESS_LOW;
    }
    else if (HAL_I2C_IsDeviceReady(&hi2c1, IMU_I2C_ADDRESS_HIGH, 2U, IMU_I2C_TIMEOUT_MS) == HAL_OK)
    {
        imu_device_address = IMU_I2C_ADDRESS_HIGH;
    }
    else
    {
        Console_Printf("IMU: device not found at address 0x68 or 0x69\r\n");
        return false;
    }

    /*
     * An ACK confirms only that some I2C device responds at the selected
     * address. Reading WHO_AM_I verifies its identity: an ICM-20948 must
     * return the fixed value 0xEA from register 0x00.
     * See WHO_AM_I, Section 8.1, page 36.
     */
    if (HAL_I2C_Mem_Read(&hi2c1, imu_device_address, IMU_REGISTER_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1U, IMU_I2C_TIMEOUT_MS) != HAL_OK)
    {
        Console_Printf("IMU: failed to read WHO_AM_I\r\n");
        return false;
    }

    Console_Printf("IMU: address=0x%02X, WHO_AM_I=0x%02X\r\n", (unsigned int)(imu_device_address >> 1U), (unsigned int)who_am_i);

    /* Reject a responding device if its identity value is not 0xEA. */
    if (who_am_i != IMU_WHO_AM_I_VALUE)
    {
        Console_Printf("IMU: unexpected WHO_AM_I value\r\n");
        return false;
    }

    Console_Printf("IMU: ICM-20948 detected\r\n");
    return true;
}

/**
 * @brief Read one raw measurement and its captured interrupt timestamp.
 *
 * ICM-20948 datasheet:
 * https://invensense.tdk.com/wp-content/uploads/2024/03/DS-000189-ICM-20948-v1.6.pdf
 *
 * @param data Pointer to the destination structure.
 * @return true if both the sensor data and interrupt timestamp are available;
 * otherwise false.
 */
bool IMU_Read(IMU_Data *data)
{
    uint8_t measurement[IMU_MEASUREMENT_SIZE];
    TimeCapture_Event interrupt_event;

    if (data == NULL)
    {
        return false;
    }

    /*
     * The measurement registers form one continuous block in user bank 0,
     * from ACCEL_XOUT_H at 0x2D through TEMP_OUT_L at 0x3A. An I2C burst
     * read automatically advances through these addresses, so all three
     * accelerometer axes, all three gyroscope axes and the temperature value
     * are acquired in one 14-byte transaction.
     * See the I2C burst read description in Section 6.4, page 30, and the
     * measurement register descriptions in Sections 8.18-8.31, pages 42-45.
     */
    if (HAL_I2C_Mem_Read(&hi2c1,
                         imu_device_address,
                         IMU_REGISTER_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT,
                         measurement,
                         IMU_MEASUREMENT_SIZE,
                         IMU_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return false;
    }

    /*
     * Each raw sensor value is a signed 16-bit quantity stored in two
     * consecutive registers, with the high byte first and the low byte
     * second. IMU_CombineBytes() shifts the high byte by 8 bits, appends the
     * low byte and interprets the resulting 16-bit pattern as int16_t.
     * See Sections 8.18-8.31, pages 42-45.
     */
    data->accelerometer_x = IMU_CombineBytes(measurement[0], measurement[1]);
    data->accelerometer_y = IMU_CombineBytes(measurement[2], measurement[3]);
    data->accelerometer_z = IMU_CombineBytes(measurement[4], measurement[5]);
    data->gyroscope_x = IMU_CombineBytes(measurement[6], measurement[7]);
    data->gyroscope_y = IMU_CombineBytes(measurement[8], measurement[9]);
    data->gyroscope_z = IMU_CombineBytes(measurement[10], measurement[11]);
    data->temperature = IMU_CombineBytes(measurement[12], measurement[13]);
    data->acceleration_magnitude_g = IMU_CalculateAccelerationMagnitude(data);
    data->valid = TimeCapture_GetImu(&interrupt_event);
    data->interrupt_capture = interrupt_event.capture;
    data->interrupt_sequence = interrupt_event.sequence;

    return data->valid;
}

/**
 * @brief Notify the IMU task that a measurement interrupt was captured.
 */
void IMU_NotifyFromISR(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (imu_task_handle == NULL)
    {
        return;
    }

    vTaskNotifyGiveFromISR(imu_task_handle, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}
