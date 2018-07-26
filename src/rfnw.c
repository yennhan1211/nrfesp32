#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "string.h"
#include "nrf24.h"
#include "server.h"
#include "rfnw.h"



#define PIN_NUM_MISO    12
#define PIN_NUM_MOSI    13
#define PIN_NUM_CLK     14
#define PIN_NUM_CS      15
#define PIN_NUM_CE      17

static xQueueHandle rfnw_send_queue = NULL;

// static uint8_t nRF24_payload[MAX_PAYLOAD_BUFFER][MAX_RF_PAYLOAD];
static uint8_t nRF24_payload[MAX_RF_PAYLOAD];
static uint8_t payload_length = 0;

static spi_device_handle_t spi;

extern uint32_t g_master_id;

static void rfnw_task_handler(void* argv);
static void rfnw_process_data(uint8_t* data, uint8_t length);
static void rfnw_process_msg_from_other_task(void);
static void rfnw_process_send_data(void);

// const uint8_t nRF24_ADDR1[5] = { 0xAB, 0xCD, 0xAB, 0xCD, 0x71 };
// const uint8_t nRF24_ADDR2[5] = { 0x54, 0x4d, 0x52, 0x68, 0x7C };

const uint8_t nRF24_ADDR1[5] = { 0x71, 0xCD, 0xAB, 0xCD, 0xAB};
const uint8_t nRF24_ADDR2[5] = { 0x7C, 0x68, 0x52, 0x4d, 0x54};

const char* RFNW_TAG = "RFNW";



static void rfnw_task_handler(void* argv)
{
    for (;;)
    {
        // Recv data from RF
        if ((nRF24_GetIRQFlags() & 0x40) != 0)
        {
          while (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY)
          {
            // Get a payload from the transceiver
            nRF24_ReadPayload(nRF24_payload, &payload_length);
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

static void rfnw_process_msg_from_other_task(void)
{

}

static uint8_t rfnw_transmit_packet(uint8_t* pack, uint8_t pack_len)
{
    nRF24_TXResult ret;
    uint8_t lret = 0;

    if(pack == NULL || pack_len == 0) return 0;

    ESP_LOGI(RFNW_TAG, "RF transmit1 %d\n", pack_len);

    nRF24_CE_Low();
    nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR1);
    nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR1);
    nRF24_SetOperationalMode(nRF24_MODE_TX);
    ret = nRF24_TransmitPacket(pack, pack_len);
    nRF24_CE_High();

    nRF24_CE_Low();
    nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR1);
    nRF24_SetOperationalMode(nRF24_MODE_RX);
    nRF24_CE_High();

    if (ret != nRF24_TX_SUCCESS)
    {
        lret = 1;
    }

    return lret;
}

static void rfnw_process_send_data(void)
{
    nrf_send_pack_t tmp_pack;

    if (pdTRUE == xQueueReceive(rfnw_send_queue, &tmp_pack, (portTickType)(10 / portTICK_RATE_MS))) {
        rfnw_transmit_packet(tmp_pack.pack_payload, tmp_pack.pack_len);
    }
    // rfnw_transmit_packet(nRF24_ADDR2, 5);
}

bool rfnw_task_send_msg(nrf_send_pack_t* msg)
{
    if(rfnw_send_queue == NULL) return false;
    if (xQueueSend(rfnw_send_queue, msg, 10 / portTICK_RATE_MS) != pdTRUE) {
        ESP_LOGE(RFNW_TAG, "%s xQueue send failed", __func__);
        return false;
    }

    return true;
}

void rfnw_prepare_pack(nrf_send_pack_t* pack, uint32_t master_id, uint32_t client_id, uint8_t type, uint8_t *pdata, uint8_t data_len)
{
    // header 1len 4master 4client 1type (data_len bytes) 1crc
    // 2 + 1+ 4 + 4+ 1 + data_len+ 1
    pack->pack_len = 2 + 1 + 4 + 4 + 1 + data_len + 1;

    pack->pack_payload[0] = 0xAA;
    pack->pack_payload[1] = 0x55;

    pack->pack_payload[2] = 1 + data_len; // 4 master + 4 client + 1 type + data_len

    // 11223344 >> 44332211

    pack->pack_payload[3] = (master_id >> 24) & 0xFF;
    pack->pack_payload[4] = (master_id >> 16) & 0xFF;
    pack->pack_payload[5] = (master_id >> 8) & 0xFF;
    pack->pack_payload[6] = (master_id >> 0) & 0xFF;

    pack->pack_payload[7] = (client_id >> 24) & 0xFF;
    pack->pack_payload[8] = (client_id >> 16) & 0xFF;
    pack->pack_payload[9] = (client_id >> 8) & 0xFF;
    pack->pack_payload[10] = (client_id >> 0) & 0xFF;

    pack->pack_payload[11] = type;

    if (pdata != NULL)
    {
        memcpy(pack->pack_payload + 12, pdata, data_len);
    }

    pack->pack_payload[data_len + 12] = rfnw_calc_crc(pack->pack_payload + 2, pack->pack_payload[2] + 9);
}

static void rfnw_process_data(uint8_t* data, uint8_t length)
{
    uint8_t* tmp_payload = NULL;
    uint8_t cmd_type = 0xFF;
    nrf_send_pack_t tmp;

    // frame: [AA 55] [1byte len] [len bytes payload] [1byte crc]
    // [len bytes payload]
    // <4 bytes master id> <4 bytes client id> <1 byte type> <n bytes data>
    // => length >= 2 + 1 + 1 + 4 + 4 + 1 + 1
    if(length >= 12 && data[0] == 0xAA && data[1] == 0x55)
    {
        // calc crc: from len to payload
        // len position: 3
        if (rfnw_calc_crc((data + 2), (data[2] + 9)) == data[length - 1])
        {

            tmp_payload = data + 3;
            cmd_type = tmp_payload[8];
            switch(cmd_type)
            {
                case CMD_READ_CLIENT_ID | 0x80:

                    tmp.pack_payload[0] = 0xAA;
                    tmp.pack_payload[1] = 0x55;
                    tmp.pack_payload[2] = 2;
                    memcpy(tmp.pack_payload + 3, data + 3, 8);
                    tmp.pack_payload[11] = CMD_READ_CLIENT_ID_ACK;
                    tmp.pack_payload[12] = 5; // 10s
                    tmp.pack_payload[13] = rfnw_calc_crc(tmp.pack_payload + 2, 11);

                    tmp.pack_len = 14;

                    // send back ack to client
                    rfnw_transmit_packet(tmp.pack_payload, tmp.pack_len);

                // tmp_pack.pack_payload[2] = tmp_pack.pack_len - 4;
                // tmp_pack.pack_payload[tmp_pack.pack_len - 1] = rfnw_calc_crc(tmp_pack.pack_payload + 2, tmp_pack.pack_payload[2] + 1);
                    //
                    tmp.pack_payload[2] = 10;
                    tmp.pack_payload[3] = SCAN_ALL_CLIENT | 0x80;
                    memcpy(tmp.pack_payload + 4, data + 3, 8);
                    tmp.pack_payload[12] =  tmp_payload[9];
                    tmp.pack_payload[13] = rfnw_calc_crc(tmp.pack_payload + 2, 11);
                    tmp.pack_len = 14;

                    // send to server task
                    server_send_msg(&tmp);
                break;

                case CMD_CONTROL | 0x80:
                    ESP_LOGI(RFNW_TAG, "CMD_CONTROL Client Id\n" );
                break;

                case CMD_READ_CLIENT_ID_ACK | 0x80:
                    ESP_LOGI(RFNW_TAG, "CMD_READ_CLIENT_ID_ACK\n");
                break;

                case CMD_CONFIG_ORDER | 0x80:
                    ESP_LOGI(RFNW_TAG, "Config order id successfully\n");
                break;

                default:
                    ESP_LOGI(RFNW_TAG, "Command type not supported\n");
                break;
            }
        }
        else
        {
            ESP_LOGI(RFNW_TAG, "Wrong checksum. Got %d - Expected %d\n",data[length - 1],  rfnw_calc_crc((data + 2), (data[2] + 1)));
        }
    }
    else
    {
        ESP_LOGI(RFNW_TAG, "Packet length is too short or wrong header or lenght not match\n");
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
    esp_err_t ret;

    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,

        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=0
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=4*1000*1000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=-1,               //CS pin
        .queue_size=2,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=NULL,                           //Specify pre-transfer callback to handle D/C line
    };

    // Init CE pin
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_NUM_CE | 1ULL << PIN_NUM_CS);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 0);
    ESP_ERROR_CHECK(ret);
    //Attach the NRF to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret)

    nRF24_Init(spi);

     nRF24_CE_Low();
    nRF24_SetRFChannel(1);
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);
    nRF24_SetDataRate(nRF24_DR_250kbps);
    nRF24_DisableAA(0xFF);
    nRF24_SetCRCScheme(nRF24_CRC_2byte);
    nRF24_SetAddrWidth(5);
    ESP_LOGI(RFNW_TAG,"nRF24_SetAddrWidth: %x\n", nRF24_ReadReg(nRF24_REG_SETUP_AW));
    nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR1); // program address for RX pipe TX
    nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR1); // program address for RX pipe #1
    nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR2); // program address for RX pipe #1
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 32); // Auto-ACK: Enable
    nRF24_SetRXPipe(nRF24_PIPE0, nRF24_AA_OFF, 32); // Auto-ACK: Enable
    //nRF24_EnableAA(nRF24_PIPE1|nRF24_PIPE0);
    nRF24_SetOperationalMode(nRF24_MODE_RX);
    nRF24_ClearIRQFlags();
    nRF24_SetPowerMode(nRF24_PWR_UP);
    //nRF24_EnableAckPayload();
    nRF24_enableDynamicPayloads(nRF24_PIPE0);
    nRF24_enableDynamicPayloads(nRF24_PIPE1);
    nRF24_CE_High();
    ESP_LOGI(RFNW_TAG,"NRF check %d\n", nRF24_Check());

    rfnw_send_queue = xQueueCreate(10, sizeof(nrf_send_pack_t));
    xTaskCreate(rfnw_task_handler, "rfnw", 3072, NULL, configMAX_PRIORITIES - 2, NULL);
    return;
}

