#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "rfnw.h"

static xQueueHandle rfnw_send_queue = NULL;
static xQueueHandle rfnw_recv_queue = NULL;
static xQueueHandle rfnw_task_queue = NULL;


static void rfnw_app_task_handler(void* argv)
{
    for (;;)
    {
        // Recv data from RF

        // Process data

        // Process msg that is sent from other task

        // Async send data to RF
    }
}

void rfnw_task_start_up(void)
{
    xTaskCreate(rfnw_app_task_handler, "rfnw", 3072, NULL, configMAX_PRIORITIES - 3, NULL);
    return;
}

