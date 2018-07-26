#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_partition.h"

#include "driver/gpio.h"

#include "esp_log.h"
#include "string.h"
#include "nrf24.h"
#include "rfnw.h"

#include "control.h"

#define PARTITION_NAME   "storage"
//sector size of flash
#define FLASH_SECTOR_SIZE         (0x1000)
//flash read / write address
#define FLASH_ADDR                (0x200000)

volatile app_context_t g_app_context = {
  .g_master_id = 0x11223344,
};

bool isControlByApp = false;
static uint32_t sync_config_timer_id = 0;
static TimerHandle_t sync_config_timer;
static uint8_t dt[MAX_LIGHT];
static void light_control_task_handler(void* argv);
static void light_control_sync_config_handler(TimerHandle_t timer);
void dump_data(void);

static void light_control_task_handler(void* argv)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1000 / portTICK_RATE_MS;
    uint32_t i = 0, j = 0, k = 0;

    for (;;)
    {
        // process
        // update state after 1 sec
        for(i = 0; i < MAX_LIGHT; i++) { // scan all lights
            for(j = 0; j < MAX_OUT; j++) { // scan all outs
                // if(p_light_context_list[i].out[j].interval == 0)
                // {
                //     p_light_context_list[i].out[j].out_state = 0;
                //     p_light_context_list[i].current_out_idx++;
                //     p_light_context_list[i].current_out_idx = p_light_context_list[i].current_out_idx % MAX_OUT;

                // }
                light_context_list_t* p_light_context_list = &(g_app_context.light_context_list[i]);
                out_t* p_j_out = &(p_light_context_list->out[j]);
                if (p_j_out->out_idx == p_light_context_list->current_out_idx) // out order = current order
                {
                    if (p_j_out->current_tick > 0)
                    {
                        p_j_out->out_state = 1;
                        p_j_out->current_tick--;
                        // printf("%d %d %d %d\n", i, j, p_light_context_list->current_out_idx, p_j_out->current_tick);
                    }
                    else // time is reached
                    {
                        p_j_out->current_tick = p_j_out->interval;
                        p_j_out->out_state = 0; // off current out
                        p_light_context_list->current_out_idx++;
                        if(p_light_context_list->current_out_idx >= MAX_OUT) p_light_context_list->current_out_idx = 0;
                        for(k = 0; k < MAX_OUT; k++) {
                            if(p_light_context_list->out[k].out_idx == p_light_context_list->current_out_idx) {
                                p_light_context_list->out[k].out_state = 1; // on next out
                                break;
                            }
                        }
                    }
                    break;
                }
            }
        }

        // broadcast to all
        if (!isControlByApp)
        {
           
            memset(dt, 0, MAX_LIGHT);
            for(i = 0; i < MAX_LIGHT; i++) {  // collect output state for all light colums
                light_context_list_t* p_light_context_list = ( light_context_list_t*)&(g_app_context.light_context_list[i]);
                for(j =0; j < MAX_OUT; j ++){
                    dt[i] |= p_light_context_list->out[j].out_state << j;
                }
            }

            dump_data();

            // control master (light colum 0)
            ligh_control_master_out(dt[0]);

            for(i = 1; i < MAX_LIGHT; i ++){ // send data to the rest
                // send to rf task
                nrf_send_pack_t tmp;
                rfnw_prepare_pack(&tmp, g_app_context.g_master_id, g_app_context.light_context_list[i].client_id,CMD_CONTROL, dt, MAX_LIGHT);
                rfnw_task_send_msg(&tmp);
                vTaskDelay(25 / portTICK_RATE_MS);
            }
        }
        // Sleep 1s
         vTaskDelayUntil( &xLastWakeTime, xFrequency);
    }
}

void ligh_control_sync_config(void)
{
    // start timer sync config
    if( xTimerIsTimerActive( sync_config_timer ) == pdFALSE )
    {
        xTimerStart(sync_config_timer, 1); // start sync timer
    }
}

void ligh_control_master_out(uint8_t data_port)
{
    gpio_set_level(OUT1_PIN, data_port & 0x01);
    gpio_set_level(OUT2_PIN, (data_port >> 1) & 0x01);
    gpio_set_level(OUT3_PIN, (data_port >> 2) & 0x01);
    // gpio_set_level(OUT4_PIN, (data_port >> 3) & 0x01);
}

static void light_control_sync_config_handler(TimerHandle_t timer)
{
    esp_err_t ret = ESP_OK;
    ESP_LOGI("Control", "Syncing config to partiton %s\n", PARTITION_NAME);
    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
            ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
    if (data_partition == NULL) {
        ESP_LOGI("Control", "Cannot find partiton %s\n", PARTITION_NAME);
        return;
    }

    ESP_LOGI("Control", "Erase size: %d Bytes\n", sizeof(app_context_t));
    ESP_ERROR_CHECK(esp_partition_erase_range(data_partition, 0, 4 * 1024));
    ret = esp_partition_write(data_partition, 0, (void*)&g_app_context, sizeof(app_context_t));

    if (ret != ESP_OK)
    {
        ESP_LOGI("Control", "Cannot write config to partiton %s\n", PARTITION_NAME);
    }
}

void dump_data(void) {
    // for(int i = 0; i < MAX_LIGHT; i++) {
    //     printf("Colum %d [\n", i);
    //     printf("current_out_idx %d\n", g_app_context.light_context_list[i].current_out_idx);
    //     printf("order_id %d\n", g_app_context.light_context_list[i].order_id);
    //     for(int j = 0; j < MAX_OUT; j++) {
    //         printf("out %d {\n", j);
    //         printf("\tout_idx %d\n", g_app_context.light_context_list[i].out[j].out_idx);
    //         printf("\tout_state %d\n", g_app_context.light_context_list[i].out[j].out_state);
    //         printf("\tcurrent_tick %d\n", g_app_context.light_context_list[i].out[j].current_tick);
    //         printf("\tinterval %d\n", g_app_context.light_context_list[i].out[j].interval);
    //         printf("}\n");
    //     }
    //     printf("]\n");
    // }
    printf("[");
    for(int i = 0; i < MAX_LIGHT; i++) printf("%d ", dt[i]);
    printf("]\n");
}

void light_control_task_start_up(void)
{
    esp_err_t ret = ESP_OK;

    // Init CE out and led pin
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << OUT1_PIN | 1ULL << OUT2_PIN | 1ULL << OUT3_PIN | 1ULL << OUT4_PIN | 1ULL << LED1_PIN | 1ULL << LED2_PIN);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Load config from flash
    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
            ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
    if (data_partition != NULL) {
        ESP_LOGI("Control", "partiton addr: 0x%08x; size: %d; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    }

    if (data_partition != NULL) {
        ret = esp_partition_read(data_partition, 0, (void*)&g_app_context, sizeof(app_context_t));
        if(ret != ESP_OK) {
            ESP_LOGI("Control", "partiton read failed\n");
        }
        else{
            if(g_app_context.g_master_id != 0xaabbccdd) {
                // prepare for fisrt time
                g_app_context.g_master_id = 0xaabbccdd;
                ESP_LOGI("Control", "Format data\n");
                for(int i = 0; i < MAX_LIGHT; i++) {
                    g_app_context.light_context_list[i].order_id = i;
                    g_app_context.light_context_list[i].client_id = 0xFFFFFFFF;
                    g_app_context.light_context_list[i].current_out_idx = 0;
                    for(int j = 0; j < MAX_OUT; j++){
                        g_app_context.light_context_list[i].out[j].out_idx = j;
                        g_app_context.light_context_list[i].out[j].interval = 0;
                        g_app_context.light_context_list[i].out[j].current_tick = 0;
                        g_app_context.light_context_list[i].out[j].out_state = 0;
                    }
                }

                ESP_LOGI("Control", "Erase size: %d Bytes\n", sizeof(app_context_t));
                ESP_ERROR_CHECK(esp_partition_erase_range(data_partition, 0, 4 * 1024));
                ret = esp_partition_write(data_partition, 0, (void*)&g_app_context, sizeof(app_context_t));
            }
        }
    }
    ESP_LOGI("Control", "Master Id: %x\n", g_app_context.g_master_id);
    for(int i = 0; i < MAX_LIGHT; i++){
        g_app_context.light_context_list[i].current_out_idx = 0;
        for(int j = 0; j < MAX_OUT; j++){
            g_app_context.light_context_list[i].out[j].current_tick = g_app_context.light_context_list[i].out[j].interval;
        }
        ESP_LOGI("Control", "Client %d Id: %x\n",i, g_app_context.light_context_list[i].client_id);
    }

    dump_data();

    sync_config_timer = xTimerCreate("sync_config_timer",10000 / portTICK_PERIOD_MS, pdFALSE,
                                         (void *)&sync_config_timer_id, light_control_sync_config_handler);

    if (sync_config_timer == NULL)
    {
        ESP_LOGI("Control", "Cannot create sync_config_timer\n");
    }

    xTaskCreate(light_control_task_handler, "ligh_control", 4096, NULL, 1, NULL);
    return;
}
