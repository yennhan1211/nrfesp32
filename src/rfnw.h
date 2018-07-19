#ifndef __RFNW_H__
#define __RFNW_H__

typedef struct rf_msg
{
  uint8_t header[2];
  uint8_t length;
  uint8_t* payload;
  uint8_t
} rf_msg_t;

void rfnw_task_start_up(void);

uint8_t rfnw_calc_crc(uint8_t* data, uint8_t length);

#endif
