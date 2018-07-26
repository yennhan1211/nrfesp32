#ifndef _CONTROL_H_
#define _CONTROL_H_

#define MAX_LIGHT            4
#define MAX_OUT              3

#define OUT1_PIN                32
#define OUT2_PIN                33
#define OUT3_PIN                25
#define OUT4_PIN                26

#define LED1_PIN                18
#define LED2_PIN                19

typedef struct {
    uint8_t out_idx;
    uint8_t out_state;
    uint8_t current_tick;
    uint8_t interval;
} out_t;

typedef struct {
    uint32_t client_id;
    uint8_t order_id;
    uint8_t current_out_idx;
    uint8_t r1;
    uint8_t r2;
    out_t out [MAX_OUT];
} light_context_list_t;

typedef struct app_context
{
    uint32_t g_master_id;
    light_context_list_t light_context_list[MAX_LIGHT];
} app_context_t;

void light_control_task_start_up(void);

void ligh_control_sync_config(void);

void ligh_control_master_out(uint8_t data_port);

#endif
