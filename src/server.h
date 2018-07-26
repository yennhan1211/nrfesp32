#ifndef __SERVER_H__
#define __SERVER_H__

#include "rfnw.h"

#define SCAN_ALL_CLIENT                 0x00
#define SET_ORDER_ID_TO_CLIENT          0x01
#define CONTROL_BY_ORDER_ID             0x02
#define SET_CONFIG                      0x03
#define READ_CONFIG                     0x04
#define OUT_CONTROL_CLIENT              0x05

void server_task_start_up(void);

bool server_send_msg(nrf_send_pack_t* msg);


#endif
