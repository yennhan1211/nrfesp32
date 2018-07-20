#ifndef _CONTROL_H_
#define _CONTROL_H_

#define MAX_LIGHT            5
#define MAX_OUT              5

typedef struct light_context {
    uint32_t client_id;
    uint8_t order_id;
    uint8_t current_out_idx;
    uint8_t r1;
    uint8_t r2;
    struct {
        uint8_t out_idx;
        uint8_t out_state;
        uint8_t current_tick;
        uint8_t interval;
    } out [MAX_OUT];
} light_context_t;

light_context_t light_context_list[MAX_LIGHT];
uint32_t g_master_id = 0x11223344;
bool isControlByApp = false;

void light_control_task_start_up(void);

#endif
