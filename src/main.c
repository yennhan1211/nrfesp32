#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"
#include "errno.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_spi_flash.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "sdkconfig.h"

#include "rfnw.h"
#include "server.h"
#include "control.h"

#define PIN_NUM_MISO    12
#define PIN_NUM_MOSI    13
#define PIN_NUM_CLK     14
#define PIN_NUM_CS      15

// #define PIN_NUM_MISO    25
// #define PIN_NUM_MOSI    23
// #define PIN_NUM_CLK     19
// #define PIN_NUM_CS      22

#define EXAMPLE_ESP_WIFI_SSID      "dengiaothong"
#define EXAMPLE_ESP_WIFI_PASS      "87654321"
#define EXAMPLE_MAX_STA_CONN       2

static const char *TAG = "nrfesp32";

volatile bool isConnected = false;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

// static spi_device_handle_t spi;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_AP_START:
        ESP_LOGI(TAG, "Access point started\n");
    break;
    // case SYSTEM_EVENT_STA_GOT_IP:
    //     ESP_LOGI(TAG, "got ip:%s",
    //              ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
    //     xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    //     break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        isConnected = true;
        ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        isConnected = false;
        ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        break;
    // case SYSTEM_EVENT_STA_DISCONNECTED:
    //     esp_wifi_connect();
    //     xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    //     break;
    default:
        ESP_LOGI(TAG, "Shouldn't be here\n");
        break;
    }
    return ESP_OK;
}

static void start_dhcp_server(){

    // initialize the tcp stack
    tcpip_adapter_init();
    // stop DHCP server
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
    // assign a static IP to the network interface
    tcpip_adapter_ip_info_t info;
    memset(&info, 0, sizeof(info));
    IP4_ADDR(&info.ip, 192, 168, 1, 1);
    IP4_ADDR(&info.gw, 192, 168, 1, 1);//ESP acts as router, so gw addr will be its own addr
    IP4_ADDR(&info.netmask, 255, 255, 255, 0);
    ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));
    // start the DHCP server
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
    ESP_LOGI(TAG,"DHCP server started \n");
}

void wifi_init_softap()
{
    wifi_event_group = xEventGroupCreate();

    start_dhcp_server();

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_OPEN
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

void app_main()
{
    esp_err_t ret;

    // spi_bus_config_t buscfg={
    //     .miso_io_num=PIN_NUM_MISO,
    //     .mosi_io_num=PIN_NUM_MOSI,
    //     .sclk_io_num=PIN_NUM_CLK,

    //     .quadwp_io_num=-1,
    //     .quadhd_io_num=-1,
    //     .max_transfer_sz=0
    // };
    // spi_device_interface_config_t devcfg={
    //     .clock_speed_hz=4*1000*1000,           //Clock out at 10 MHz
    //     .mode=0,                                //SPI mode 0
    //     .spics_io_num=-1,               //CS pin
    //     .queue_size=2,                          //We want to be able to queue 7 transactions at a time
    //     .pre_cb=NULL,                           //Specify pre-transfer callback to handle D/C line
    // };

    //Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // for (int i = 10; i >= 0; i--) {
    //     gpio_set_level(16, i%2);
    //     printf("Restarting in %d seconds...\n", i);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    light_control_task_start_up();
    rfnw_task_start_up();
    server_task_start_up();

    // printf("Restarting now.\n");
    // fflush(stdout);
    // esp_restart();

}
