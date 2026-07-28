/**
 * @file gnss.c
 * @brief GNSS module implementation.
 */

#include "gnss.h"

#include "time_capture.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/** Size of the circular USART2 DMA receive buffer. */
#define GNSS_RX_DMA_BUFFER_SIZE 1024U

/** Maximum length of one NMEA sentence, including line termination. */
#define GNSS_NMEA_SENTENCE_SIZE 128U

/**
 * @brief Indices of the GGA fields used by the parser.
 */
typedef enum
{
    GNSS_GGA_FIELD_MESSAGE = 0,
    GNSS_GGA_FIELD_TIME,
    GNSS_GGA_FIELD_LATITUDE,
    GNSS_GGA_FIELD_LATITUDE_DIRECTION,
    GNSS_GGA_FIELD_LONGITUDE,
    GNSS_GGA_FIELD_LONGITUDE_DIRECTION,
    GNSS_GGA_FIELD_FIX_QUALITY,
    GNSS_GGA_FIELD_COUNT
} GNSS_GgaFieldIndex;

/** Circular DMA buffer used to receive GNSS NMEA data. */
static uint8_t gnss_rx_dma_buffer[GNSS_RX_DMA_BUFFER_SIZE];

/** Buffer used to assemble one complete NMEA sentence. */
static char gnss_nmea_sentence[GNSS_NMEA_SENTENCE_SIZE];

/** Handle of the task responsible for GNSS communication and processing. */
static TaskHandle_t gnss_task_handle;

/** Current USART2 DMA write position reported by the HAL callback. */
static volatile uint16_t gnss_dma_write_position;

/** Current write position in the NMEA sentence buffer. */
static uint16_t gnss_nmea_position;

/** Indicates that an NMEA sentence is currently being assembled. */
static bool gnss_nmea_active;

/** Latest valid GNSS position associated with a PPS edge. */
static GNSS_Fix gnss_latest_fix;

/**
 * @brief Parse a complete GGA sentence and print valid coordinates.
 *
 * @param sentence Null-terminated NMEA sentence.
 */
static void GNSS_ParseGga(char *sentence)
{
    char *fields[GNSS_GGA_FIELD_COUNT];
    GNSS_Fix new_fix = {0};
    TimeCapture_Event pps;
    uint8_t field_count = 1U;

    if (strncmp(sentence, "$GNGGA,", 7U) != 0)
    {
        return;
    }

    fields[0] = sentence;

    for (char *character = sentence; (*character != '\0') && (field_count < GNSS_GGA_FIELD_COUNT); character++)
    {
        if (*character == ',')
        {
            *character = '\0';
            fields[field_count] = character + 1;
            field_count++;
        }
    }

    if ((field_count < GNSS_GGA_FIELD_COUNT) ||
        (fields[GNSS_GGA_FIELD_LATITUDE][0] == '\0') ||
        (fields[GNSS_GGA_FIELD_LATITUDE_DIRECTION][0] == '\0') ||
        (fields[GNSS_GGA_FIELD_LONGITUDE][0] == '\0') ||
        (fields[GNSS_GGA_FIELD_LONGITUDE_DIRECTION][0] == '\0') ||
        (fields[GNSS_GGA_FIELD_FIX_QUALITY][0] == '0'))
    {
        return;
    }

    strncpy(new_fix.utc_time, fields[GNSS_GGA_FIELD_TIME], GNSS_UTC_TIME_SIZE - 1U);
    strncpy(new_fix.latitude, fields[GNSS_GGA_FIELD_LATITUDE], GNSS_COORDINATE_SIZE - 1U);
    strncpy(new_fix.longitude, fields[GNSS_GGA_FIELD_LONGITUDE], GNSS_COORDINATE_SIZE - 1U);
    new_fix.latitude_direction = fields[GNSS_GGA_FIELD_LATITUDE_DIRECTION][0];
    new_fix.longitude_direction = fields[GNSS_GGA_FIELD_LONGITUDE_DIRECTION][0];

    new_fix.valid = TimeCapture_GetPps(&pps);
    new_fix.pps_capture = pps.capture;
    new_fix.pps_sequence = pps.sequence;

    taskENTER_CRITICAL();
    gnss_latest_fix = new_fix;
    taskEXIT_CRITICAL();

    if (new_fix.valid)
    {
        printf("GNSS fix: UTC=%s, PPS=%lu, position=%s %c, %s %c\r\n", new_fix.utc_time, (unsigned long)new_fix.pps_sequence, new_fix.latitude, new_fix.latitude_direction, new_fix.longitude, new_fix.longitude_direction);
    }
}

/**
 * @brief Add one received byte to the current NMEA sentence.
 *
 * @param received_byte Byte received from the GNSS module.
 */
static void GNSS_ProcessByte(uint8_t received_byte)
{
    if (received_byte == (uint8_t)'$')
    {
        gnss_nmea_position = 0U;
        gnss_nmea_active = true;
    }

    if (!gnss_nmea_active)
    {
        return;
    }

    if (gnss_nmea_position >= (GNSS_NMEA_SENTENCE_SIZE - 1U))
    {
        gnss_nmea_position = 0U;
        gnss_nmea_active = false;
        return;
    }

    gnss_nmea_sentence[gnss_nmea_position] = (char)received_byte;
    gnss_nmea_position++;

    if (received_byte == (uint8_t)'\n')
    {
        gnss_nmea_sentence[gnss_nmea_position] = '\0';
        gnss_nmea_position = 0U;
        gnss_nmea_active = false;
        GNSS_ParseGga(gnss_nmea_sentence);
    }
}

/**
 * @brief Process a contiguous section of the GNSS DMA buffer.
 *
 * @param start_position Index of the first byte to print.
 * @param end_position Index immediately after the final byte to process.
 */
static void GNSS_ProcessDmaSection(uint16_t start_position, uint16_t end_position)
{
    for (uint16_t position = start_position; position < end_position; position++)
    {
        GNSS_ProcessByte(gnss_rx_dma_buffer[position]);
    }
}

/**
 * @brief Run the GNSS communication task.
 *
 * Starts continuous USART2 DMA reception. Received GGA sentences are parsed
 * to extract valid GNSS coordinates.
 *
 * @param argument Unused task argument.
 */
static void GNSS_Task(void *argument)
{
    uint16_t dma_read_position = 0U;
    uint16_t dma_write_position;

    (void)argument;

    printf("GNSS: starting UART DMA reception\r\n");

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, gnss_rx_dma_buffer, GNSS_RX_DMA_BUFFER_SIZE) != HAL_OK)
    {
        printf("GNSS: failed to start UART DMA reception\r\n");
        vTaskDelete(NULL);
        return;
    }

    for (;;)
    {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        dma_write_position = gnss_dma_write_position;

        if (dma_write_position == dma_read_position)
        {
            continue;
        }

        if (dma_write_position > dma_read_position)
        {
            GNSS_ProcessDmaSection(dma_read_position, dma_write_position);
        }
        else
        {
            GNSS_ProcessDmaSection(dma_read_position, GNSS_RX_DMA_BUFFER_SIZE);
            GNSS_ProcessDmaSection(0U, dma_write_position);
        }

        dma_read_position = dma_write_position;
    }
}

/**
 * @brief Create the task responsible for GNSS operation.
 *
 * @return true if the GNSS task was created successfully; otherwise false.
 */
bool GNSS_Init(void)
{
    return (xTaskCreate(GNSS_Task,
                        "GNSS",
                        512U,
                        NULL,
                        tskIDLE_PRIORITY + 1U,
                        &gnss_task_handle) == pdPASS);
}

/**
 * @brief Copy the latest valid GNSS fix.
 *
 * @param fix Pointer to the destination structure.
 * @return true if a valid GNSS fix is available; otherwise false.
 */
bool GNSS_GetLatestFix(GNSS_Fix *fix)
{
    if (fix == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    *fix = gnss_latest_fix;
    taskEXIT_CRITICAL();

    return fix->valid;
}

/**
 * @brief Handle a USART receive-to-idle DMA event.
 *
 * For circular DMA reception, Size is the current write position inside the
 * DMA buffer. The GNSS task compares it with its previous read position and
 * handles counter wraparound.
 *
 * @param huart Pointer to the UART handle that generated the event.
 * @param Size Current DMA write position.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if ((huart == NULL) || (huart->Instance != USART2))
    {
        return;
    }

    gnss_dma_write_position = (Size < GNSS_RX_DMA_BUFFER_SIZE) ? Size : 0U;

    vTaskNotifyGiveFromISR(gnss_task_handle, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}
