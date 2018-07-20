#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "string.h"
#include "nrf24.h"
#include "rfnw.h"

#include "control.h"

static void light_control_task_handler(void* argv);

static void light_control_task_handler(void* argv)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1000 / portTICK_RATE_MS;
    uint32_t i = 0, j = 0;

    for (;;)
    {
        // process
        // update state after 1 sec
        for(i = 0; i < MAX_LIGHT; i++) { // scan all lights
            for(j = 0; j < MAX_OUT; j++) { // scan all outs
                if (light_context_list[i].out[j].out_idx == light_context_list[i].current_out_idx) // out order = current order
                {
                    if (light_context_list[i].out[j].current_tick > 0)
                    {
                        light_context_list[i].out[j].current_tick--;
                    }
                    else // time is reached
                    {
                        light_context_list[i].out[j].current_tick = light_context_list[i].out[j].interval;
                        light_context_list[i].out[j].out_state = 0; // off current out
                        light_context_list[i].current_out_idx++;
                        light_context_list[i].current_out_idx = light_context_list[i].current_out_idx % MAX_OUT;
                        light_context_list[i].out[light_context_list[i].current_out_idx].out_state = 1; // on next out
                    }
                }
            }
        }
        // broadcast to all
        if (!isControlByApp)
        {
            
        }
        // Sleep 1s
         vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}

void light_control_task_start_up(void)
{
    // uint32_t i = 0, j = 0;

    // Load config from flash

    // Setup out index

    xTaskCreate(light_control_task_handler, "ligh_control", 3072, NULL, configMAX_PRIORITIES - 1, NULL);
    return;
}
