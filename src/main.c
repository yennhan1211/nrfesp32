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

#include "nrf24.h"

#define PIN_NUM_MISO    12
#define PIN_NUM_MOSI    13
#define PIN_NUM_CLK     14
#define PIN_NUM_CS      15

// #define PIN_NUM_MISO    25
// #define PIN_NUM_MOSI    23
// #define PIN_NUM_CLK     19
// #define PIN_NUM_CS      22

#define EXAMPLE_ESP_WIFI_SSID      "nrfesp32"
#define EXAMPLE_ESP_WIFI_PASS      "12345678"
#define EXAMPLE_MAX_STA_CONN       2

static const char *TAG = "nrfesp32";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

static spi_device_handle_t spi;

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
        ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
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
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

char recv_buf[256];

void socket_server(void *ignore) {
	fd_set readfds;
	fd_set errfds;
    struct timeval select_timeout;
    struct timeval receiving_timeout;
    int flags = 0;
    int ret = 0;
    struct sockaddr_in clientAddress;
    struct sockaddr_in serverAddress;

    receiving_timeout.tv_sec = 0;
    receiving_timeout.tv_usec = 100000;

    ESP_LOGI(TAG, "Enter socket server\n");
    // Create a socket that we will listen upon.
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGI(TAG, "socket: %d %s", sock, strerror(errno));
        goto END;
    }

    // Bind our server socket to a port.
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddress.sin_port = htons(6969);
    int rc  = bind(sock, (struct sockaddr *)&serverAddress, sizeof(serverAddress));
    if (rc < 0) {
        ESP_LOGI(TAG, "bind: %d %s", rc, strerror(errno));
        goto END;
    }

    // Flag the socket as listening for new connections.
    rc = listen(sock, 1);
    if (rc < 0) {
        ESP_LOGI(TAG, "listen: %d %s", rc, strerror(errno));
        goto END;
    }

    while (1) {
        // Listen for a new client connection.
        socklen_t clientAddressLength = sizeof(clientAddress);
        int clientSock = accept(sock, (struct sockaddr *)&clientAddress, &clientAddressLength);
        if (clientSock < 0) {
            ESP_LOGI(TAG, "accept: %d %s", clientSock, strerror(errno));
            goto END;
        }

        ESP_LOGI(TAG, "New client connection: fd: %d, url: %s:%d %d", clientSock, inet_ntoa(clientAddress.sin_addr) , ntohs(clientAddress.sin_port), sizeof(recv_buf));

        if (setsockopt(clientSock, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(clientSock);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... set socket receiving timeout success");

        while(1) { // Only break this while when has error
            FD_ZERO(&readfds);
            FD_CLR( clientSock, &readfds);
            FD_SET( clientSock, &readfds);
            FD_ZERO(&errfds);
            FD_CLR( clientSock, &errfds);
            FD_SET( clientSock, &errfds);
            // Monitor fd
            ret = select( clientSock , &readfds , NULL , &errfds , &select_timeout);

            if(ret > 0) {
                // Check fd error
                if (FD_ISSET(clientSock, &errfds)) {
                    ESP_LOGI(TAG, "Has problem on client socket %d. CLOSE IT", clientSock);
                    close(clientSock);
                    break;
                }
                

                // Reading data
                if (FD_ISSET(clientSock, &errfds)) {
                    ret = recv(clientSock, recv_buf, sizeof(recv_buf), 0);
                    if(ret > 0) {
                        ESP_LOGI(TAG, "Message: %s\n", recv_buf);
                    }
                }
            } else if (ret < 0) {
                ESP_LOGI(TAG, "Has problem on client socket %d. CLOSE IT", clientSock);
                close(clientSock);
                break;
            } else {
                // Timeout
                vTaskDelay(10 / portTICK_RATE_MS);
            }

            // Something need to send out 
        }

        // Clean res
    }
    END:
    ESP_LOGI(TAG, "Exit socket server\n");
    vTaskDelete(NULL);
}

void app_main()
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

    //Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    // Init CE pin
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << 17 | 1ULL << 16 | 1ULL << PIN_NUM_CS);
    // io_conf.pin_bit_mask = (1ULL << 17 | 1ULL << 16);
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

    printf("NRF check %d\n", nRF24_Check());

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

    for (int i = 10; i >= 0; i--) {
        gpio_set_level(16, i%2);
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

     xTaskCreate(&socket_server,"socket_server",4096,NULL,5,NULL);
    // printf("Restarting now.\n");
    // fflush(stdout);
    // esp_restart();
}