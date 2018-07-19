#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "rfnw.h"

#define MAX_RF_PAYLOAD                      32
#define CMD_CONFIG_ORDER                    0x00
#define CMD_CONTROL                         0x02
#define CMD_READ_CLIENT_ID                  0x01
#define CMD_SCAN_ID                         0x03

static xQueueHandle rfnw_send_queue = NULL;
static xQueueHandle rfnw_recv_queue = NULL;

static uint8_t nRF24_payload[MAX_RF_PAYLOAD];
static uint8_t payload_length = 0;

static void rfnw_app_task_handler(void* argv);
static void rfnw_process_data(uint8_t* data, uint8_t length);

const char* RFNW_TAG = "RFNW";

static void rfnw_app_task_handler(void* argv)
{
    for (;;)
    {
        // Recv data from RF
        if ((nRF24_GetIRQFlags() & 0x40) != 0)
        {
          while (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY)
          {
            // Get a payload from the transceiver
            uint8_t pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);
            // Clear all pending IRQ flags
            nRF24_ClearIRQFlags();
          }
        }

        // Process data
        if (payload_length > 0)
        {
            rfnw_process_data(nRF24_payload, payload_length);
            // Clean rf data
            payload_length = 0;
            memset(nRF24_payload, 0, MAX_RF_PAYLOAD);
        }

        // Process msg that is sent from other task
        rfnw_process_msg_from_other_task();

        // Async send data to RF
        rfnw_process_send_data();
    }
}

static void rfnw_process_msg_from_other_task(voidd)
{

}

static void rfnw_process_send_data(void)
{

}

static void rfnw_process_data(uint8_t* data, uint8_t length)
{
    uint8_t tmp_payload_len = 0;
    uint8_t* tmp_payload = NULL;
    uint8_t cmd_type = 0xFF;

    // frame: [AA 55] [1byte len] [len bytes payload] [1byte crc]
    // 2 header + 1 len + n payload + 1 crc => length = 2 + 1 + data[2] + 1
    if(length >= 5 && data[0] == 0xAA && data[1] == 0x55 && length == (4 + data[2]))
    {
        // calc crc: from len to payload
        // len position: 2
        if (rfnw_calc_crc((data + 2), (data[2] + 1)) == data[length - 1])
        {
            tmp_payload = data + 3;
            tmp_payload_len = data[2];
            cmd_type = tmp_payload[0];
            switch(cmd_type)
            {
                case CMD_READ_CLIENT_ID:
                break;

                case CMD_CONTROL:
                break;

                case CMD_SCAN_ID:
                break;

                case CMD_CONFIG_ORDER:
                break;

                default:
                    ESP_LOGI(RFNW_TAG, "Command type not supported\n");
                break;
            }
        }
    }
}

uint8_t rfnw_calc_crc(uint8_t* data, uint8_t length)
{
    uint8_t i = 0;
    uint8_t crc = 0;

    for(i = 0; i < length; i++)
    {
        crc += data[i];
    }

    crc &= 0xFF;

    return crc;
}

void rfnw_task_start_up(void)
{
    xTaskCreate(rfnw_app_task_handler, "rfnw", 3072, NULL, configMAX_PRIORITIES - 3, NULL);
    return;
}

