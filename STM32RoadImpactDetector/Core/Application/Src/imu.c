/**
 * @file imu.c
 * @brief ICM-20948 IMU module implementation.
 */

#include "imu.h"

#include "i2c.h"
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

/** Gyroscope sample-rate divider for approximately 10 Hz output. */
#define IMU_GYROSCOPE_SAMPLE_RATE_DIVIDER 109U

/** Accelerometer sample-rate divider for approximately 10 Hz output. */
#define IMU_ACCELEROMETER_SAMPLE_RATE_DIVIDER 112U

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
 * @param data Pointer to a raw IMU measurement.
 * @return Total acceleration magnitude in g.
 */
static float IMU_CalculateAccelerationMagnitude(const IMU_Data *data)
{
    float magnitude;

    magnitude = sqrtf((float)data->accelerometer_x * data->accelerometer_x +
                      (float)data->accelerometer_y * data->accelerometer_y +
                      (float)data->accelerometer_z * data->accelerometer_z);

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
 * approximately 10 Hz. A rising edge is generated on INT when new raw sensor
 * data are ready.
 *
 * @return true if all configuration registers were written successfully;
 * otherwise false.
 */
static bool IMU_Configure(void)
{
    uint8_t accelerometer_config;
    uint8_t accelerometer_range_selection;

    if (!IMU_SelectRegisterBank(IMU_REGISTER_BANK_0))
    {
        return false;
    }

    if (!IMU_WriteRegister(IMU_REGISTER_PWR_MGMT_1, 0x01U))
    {
        return false;
    }

    if (!IMU_WriteRegister(IMU_REGISTER_PWR_MGMT_2, 0x00U))
    {
        return false;
    }

    if (!IMU_SelectRegisterBank(IMU_REGISTER_BANK_2))
    {
        return false;
    }

    if (!IMU_WriteRegister(IMU_REGISTER_GYRO_SMPLRT_DIV, IMU_GYROSCOPE_SAMPLE_RATE_DIVIDER))
    {
        return false;
    }

    if (!IMU_WriteRegister(IMU_REGISTER_ACCEL_SMPLRT_DIV_1, 0x00U))
    {
        return false;
    }

    if (!IMU_WriteRegister(IMU_REGISTER_ACCEL_SMPLRT_DIV_2, IMU_ACCELEROMETER_SAMPLE_RATE_DIVIDER))
    {
        return false;
    }

    if (!IMU_ReadRegister(IMU_REGISTER_ACCEL_CONFIG, &accelerometer_config))
    {
        return false;
    }

    accelerometer_range_selection = (accelerometer_config & IMU_ACCELEROMETER_RANGE_MASK) >> IMU_ACCELEROMETER_RANGE_POSITION;
    imu_accelerometer_full_scale_g = (uint8_t)(2U << accelerometer_range_selection);

    if (!IMU_SelectRegisterBank(IMU_REGISTER_BANK_0))
    {
        return false;
    }

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
    float acceleration_magnitude;
    uint32_t acceleration_magnitude_hundredths;

    (void)argument;

    if (!IMU_Test())
    {
        vTaskDelete(NULL);
        return;
    }

    if (!IMU_Configure())
    {
        printf("IMU: configuration failed\r\n");
        vTaskDelete(NULL);
        return;
    }

    printf("IMU: measurement interrupt enabled\r\n");
    printf("IMU: accelerometer range=+/-%u g\r\n", imu_accelerometer_full_scale_g);

    for (;;)
    {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (IMU_Read(&data))
        {
            acceleration_magnitude = IMU_CalculateAccelerationMagnitude(&data);
            acceleration_magnitude_hundredths = (uint32_t)(acceleration_magnitude * 100.0f + 0.5f);

            printf("IMU: INT=%lu, ACC=[%d, %d, %d], |ACC|=%lu.%02lu g, GYRO=[%d, %d, %d], TEMP=%d\r\n",
                   (unsigned long)data.interrupt_sequence,
                   data.accelerometer_x,
                   data.accelerometer_y,
                   data.accelerometer_z,
                   (unsigned long)(acceleration_magnitude_hundredths / 100U),
                   (unsigned long)(acceleration_magnitude_hundredths % 100U),
                   data.gyroscope_x,
                   data.gyroscope_y,
                   data.gyroscope_z,
                   data.temperature);
        }
        else
        {
            printf("IMU: measurement read failed\r\n");

            while (!IMU_Recover())
            {
                printf("IMU: recovery failed, retrying\r\n");
                vTaskDelay(pdMS_TO_TICKS(IMU_RECOVERY_DELAY_MS));
            }

            printf("IMU: communication recovered\r\n");
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
                        256U,
                        NULL,
                        tskIDLE_PRIORITY + 1U,
                        &imu_task_handle) == pdPASS);
}

/**
 * @brief Check the ICM-20948 I2C address and WHO_AM_I register.
 *
 * @return true if an ICM-20948 is detected; otherwise false.
 */
bool IMU_Test(void)
{
    uint8_t who_am_i;

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
        printf("IMU: device not found at address 0x68 or 0x69\r\n");
        return false;
    }

    if (HAL_I2C_Mem_Read(&hi2c1, imu_device_address, IMU_REGISTER_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1U, IMU_I2C_TIMEOUT_MS) != HAL_OK)
    {
        printf("IMU: failed to read WHO_AM_I\r\n");
        return false;
    }

    printf("IMU: address=0x%02X, WHO_AM_I=0x%02X\r\n", (unsigned int)(imu_device_address >> 1U), (unsigned int)who_am_i);

    if (who_am_i != IMU_WHO_AM_I_VALUE)
    {
        printf("IMU: unexpected WHO_AM_I value\r\n");
        return false;
    }

    printf("IMU: ICM-20948 detected\r\n");
    return true;
}

/**
 * @brief Read one raw measurement and its captured interrupt timestamp.
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

    data->accelerometer_x = IMU_CombineBytes(measurement[0], measurement[1]);
    data->accelerometer_y = IMU_CombineBytes(measurement[2], measurement[3]);
    data->accelerometer_z = IMU_CombineBytes(measurement[4], measurement[5]);
    data->gyroscope_x = IMU_CombineBytes(measurement[6], measurement[7]);
    data->gyroscope_y = IMU_CombineBytes(measurement[8], measurement[9]);
    data->gyroscope_z = IMU_CombineBytes(measurement[10], measurement[11]);
    data->temperature = IMU_CombineBytes(measurement[12], measurement[13]);
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
