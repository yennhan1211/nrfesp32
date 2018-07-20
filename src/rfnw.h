#ifndef __RFNW_H__
#define __RFNW_H__

#define MAX_RF_PAYLOAD                      32
#define MAX_PAYLOAD_BUFFER                  4

#define LENGTH_POS                          2

#define CMD_CONFIG_ORDER                    0x00
#define CMD_CONTROL                         0x02
#define CMD_READ_CLIENT_ID                  0x01
#define CMD_READ_CLIENT_ID_ACK              0x03

typedef struct rf_msg
{
  uint8_t header[2];
  uint8_t length;
  uint8_t* payload;
} rf_msg_t;

typedef struct nrf_send_pack
{
    uint8_t pack_len;
    uint8_t pack_payload[MAX_RF_PAYLOAD];
} nrf_send_pack_t;

void rfnw_task_start_up(void);

bool rfnw_task_send_msg(nrf_send_pack_t* msg);

uint8_t rfnw_calc_crc(uint8_t* data, uint8_t length);

#endif
