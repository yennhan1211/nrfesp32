#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "errno.h"
#include "string.h"
#include "esp_log.h"

#include "server.h"




const char* SERVER_TAG = "SERVER";

static xQueueHandle server_rfms_queue = NULL;

static void server_task_handler(void* argv);
static void server_process_data(int s, uint8_t* data, uint8_t len);
static void prepare_pack(nrf_send_pack_t* pack,uint32_t client_id, uint8_t type, uint8_t *pdata, uint8_t data_len);

static uint8_t recv_buf[256];
static uint8_t payload[128];
static uint8_t state = 0;
static uint8_t r_length = 0;
static uint8_t r_length_count = 0;
static uint8_t r_crc = 0;

extern uint32_t g_master_id;
extern bool isControlByApp;

void server_task_start_up(void)
{
    server_rfms_queue = xQueueCreate(10, sizeof(nrf_send_pack_t));
    xTaskCreate(server_task_handler, "server", 3072, NULL, configMAX_PRIORITIES - 3, NULL);
    return;
}

bool server_send_msg(nrf_send_pack_t* msg)
{
    if (xQueueSend(server_rfms_queue, msg, 10 / portTICK_RATE_MS) != pdTRUE) {
        ESP_LOGE(SERVER_TAG, "%s xQueue send failed", __func__);
        return false;
    }

    return true;
}

static void prepare_pack(nrf_send_pack_t* pack,uint32_t client_id, uint8_t type, uint8_t *pdata, uint8_t data_len)
{
    uint32_t dummy_client = client_id;

    // 2header 1len 4master 4client 1type (data_len bytes) 1crc
    // 2 + 1+ 4 + 4+ 1 + data_len+ 1
    pack->pack_len = 2 + 1 + 4 + 4 + 1 + data_len + 1;

    pack->pack_payload[0] = 0xAA;
    pack->pack_payload[1] = 0x55;

    pack->pack_payload[2] = 1 + data_len; // 4 master + 4 client + 1 type + data_len

    memcpy(pack->pack_payload + 3, &g_master_id, 4);

    memcpy(pack->pack_payload + 7, &dummy_client, 4);

    pack->pack_payload[11] = type;

    if (pdata != NULL)
    {
        memcpy(pack->pack_payload + 12, pdata, data_len);
    }

    pack->pack_payload[data_len + 12] = rfnw_calc_crc(pack->pack_payload + 2, pack->pack_payload[2] + 9);
}

static void server_process_data(int s,uint8_t* data, uint8_t len)
{
    uint8_t i = 0;
    nrf_send_pack_t tmp;
    for(i = 0; i < len; i++) {
        switch(state)
        {
            case 0: if (data[i] == 0xAA) state++; break;
            case 1: 
                if (data[i] == 0x55) 
                {
                    state++; 
                } else {
                    ESP_LOGI(SERVER_TAG, "Expect next header byte 55\n");
                    state = 0;
                }
                break;
            case 2: state++; r_length = data[i]; r_crc += r_length; break;
            case 3:
                if (r_length_count ==  (r_length - 1)) state++;
                payload[r_length_count] = data[i];
                r_crc += data[i];
                r_length_count++;
                break;
            case 4:
                if (r_crc == data[i])
                {
                    // do action
                    switch(payload[0])
                    {
                        case SCAN_ALL_CLIENT:
                            prepare_pack(&tmp, 0, CMD_READ_CLIENT_ID, NULL, 0);
                            rfnw_task_send_msg(&tmp);
                            break;
                        case SET_ORDER_ID_TO_CLIENT:
                            prepare_pack(&tmp, *(uint32_t*)(payload + 1), CMD_CONFIG_ORDER, NULL, 0);
                            rfnw_task_send_msg(&tmp);
                            break;
                        case CONTROL_BY_ORDER_ID:
                            isControlByApp = true;
                            break;
                        case SET_CONFIG:
                            break;
                        case READ_CONFIG:
                            break;
                        case OUT_CONTROL_CLIENT:
                            isControlByApp = false;
                            break;
                        default:
                            ESP_LOGI(SERVER_TAG, "Shouldn't be here: payload[0] = %d\n", payload[0]);
                            break;
                    }
                }
                else
                {
                    ESP_LOGI(SERVER_TAG, "Wrong checksum: got %d expected %d\n", r_crc, data[i]);
                }
                r_length = r_length_count = state = r_crc = 0;
                memset(payload, 0, 128);
                break;
            default:
                ESP_LOGI(SERVER_TAG, "Shouldn't be here: state = %d\n", state);
                break;
        }
    }
}

static void server_task_handler(void* argv)
{
    fd_set readfds;
    fd_set errfds;
    int ret = 0;
    struct sockaddr_in clientAddress;
    struct sockaddr_in serverAddress;

    ESP_LOGI(SERVER_TAG, "Enter socket server task\n");

    // Create a socket that we will listen upon.
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGI(SERVER_TAG, "socket: %d %s", sock, strerror(errno));
        goto END;
    }

    // Bind our server socket to a port.
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddress.sin_port = htons(6969);

    int rc  = bind(sock, (struct sockaddr *)&serverAddress, sizeof(serverAddress));
    if (rc < 0) {
        ESP_LOGI(SERVER_TAG, "bind: %d %s", rc, strerror(errno));
        goto END;
    }

    // Flag the socket as listening for new connections.
    rc = listen(sock,5);
    if (rc < 0) {
        ESP_LOGI(SERVER_TAG, "listen: %d %s", rc, strerror(errno));
        goto END;
    }

    while (1) {
        // Listen for a new client connection.
        socklen_t clientAddressLength = sizeof(clientAddress);
        int clientSock = accept(sock, (struct sockaddr *)&clientAddress, &clientAddressLength);
        if (clientSock < 0) {
            ESP_LOGI(SERVER_TAG, "accept: %d %s", clientSock, strerror(errno));
            goto END;
        }

        ESP_LOGI(SERVER_TAG, "New client connection: fd: %d, url: %s:%d", clientSock, inet_ntoa(clientAddress.sin_addr) , ntohs(clientAddress.sin_port));

        while(1)
        {
            // Only break this while when has error
            FD_ZERO(&readfds);
            FD_SET( clientSock, &readfds);
            FD_ZERO(&errfds);
            FD_SET( clientSock, &errfds);
            struct timeval select_timeout;
            select_timeout.tv_sec = 0;
            select_timeout.tv_usec = 50000;
            // Monitor fd
            ret = select( clientSock + 1, &readfds , NULL , &errfds , &select_timeout);

            if(ret > 0) {
                // Check fd error
                if (FD_ISSET(clientSock, &errfds)) {
                    ESP_LOGI(SERVER_TAG, "Has problem on client socket %d. CLOSE IT", clientSock);
                    close(clientSock);
                    break;
                }

                // Reading data
                if (FD_ISSET(clientSock, &readfds)) {
                    ret = recv(clientSock, recv_buf, sizeof(recv_buf), 0);

                    if (ret < 0) // has some errors
                    {
                        close(clientSock);
                        ESP_LOGI(SERVER_TAG, "Has problem on client socket %d. CLOSE IT", clientSock);
                        break;
                    }
                    else if (ret == 0)
                    {
                        // socket close by remote
                        close(clientSock);
                        ESP_LOGI(SERVER_TAG, "socket close %d", clientSock);
                        break;
                    }
                    else if(ret > 0) {
                        // process incoming data
                        ESP_LOGI(SERVER_TAG, "Message: %s\n", recv_buf);
                        server_process_data(clientSock, recv_buf, ret);
                    }
                }
            } else if (ret < 0) {
                ESP_LOGI(SERVER_TAG, "Has problem on client socket %d. CLOSE IT", clientSock);
                close(clientSock);
                break;
            } else {
                // Timeout
                vTaskDelay(10 / portTICK_RATE_MS);
            }
            // Something need to send out
            nrf_send_pack_t tmp_pack;
            if (pdTRUE == xQueueReceive(server_rfms_queue, &tmp_pack, (portTickType)(10 / portTICK_RATE_MS))) {
                ESP_LOGI(SERVER_TAG, "Send to socket server task %d\n", tmp_pack.pack_len);
                // tmp_pack.pack_payload[2] = tmp_pack.pack_len - 4;
                // tmp_pack.pack_payload[tmp_pack.pack_len - 1] = rfnw_calc_crc(tmp_pack.pack_payload + 2, tmp_pack.pack_payload[2] + 1);
                write(clientSock, tmp_pack.pack_payload, tmp_pack.pack_len);
            }
        }
        // Clean res
    }

END:
    ESP_LOGI(SERVER_TAG, "Exit socket server task\n");
    vTaskDelete(NULL);
}
