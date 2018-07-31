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
#include "control.h"




const char* SERVER_TAG = "SERVER";

static xQueueHandle server_rfms_queue = NULL;

static void server_task_handler(void* argv);
static void server_process_data(int s, uint8_t* data, uint8_t len);

static uint8_t recv_buf[256];
static uint8_t payload[128];
static uint8_t state = 0;
static uint8_t r_length = 0;
static uint8_t r_length_count = 0;
static uint8_t r_crc = 0;

extern app_context_t g_app_context;
extern bool isControlByApp;
extern volatile bool isConnected;

void server_task_start_up(void)
{
    server_rfms_queue = xQueueCreate(10, sizeof(nrf_send_pack_t));
    xTaskCreate(server_task_handler, "server", 3072, NULL, configMAX_PRIORITIES - 3, NULL);
    return;
}

bool server_send_msg(nrf_send_pack_t* msg)
{   if(server_rfms_queue == NULL) return false;
    if (xQueueSend(server_rfms_queue, msg, 10 / portTICK_RATE_MS) != pdTRUE) {
        ESP_LOGE(SERVER_TAG, "%s xQueue send failed", __func__);
        return false;
    }

    return true;
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
                            ESP_LOGI(SERVER_TAG, "Scan client\n");
                            isControlByApp = true;
                            // send its own
                            uint8_t count = 0;
                            tmp.pack_payload[count++] = 0xAA;
                            tmp.pack_payload[count++] = 0x55;
                            tmp.pack_payload[count++] = 10;
                            tmp.pack_payload[count++] = SCAN_ALL_CLIENT | 0x80;
                            tmp.pack_payload[count++] = (g_app_context.g_master_id >> 24) & 0xFF;
                            tmp.pack_payload[count++] = (g_app_context.g_master_id >> 16) & 0xFF;
                            tmp.pack_payload[count++] = (g_app_context.g_master_id >> 8) & 0xFF;
                            tmp.pack_payload[count++] = (g_app_context.g_master_id >> 0) & 0xFF;
                            tmp.pack_payload[count++] = (g_app_context.light_context_list[0].client_id >> 24) & 0xFF;
                            tmp.pack_payload[count++] = (g_app_context.light_context_list[0].client_id >> 16) & 0xFF;
                            tmp.pack_payload[count++] = (g_app_context.light_context_list[0].client_id >> 8) & 0xFF;
                            tmp.pack_payload[count++] = (g_app_context.light_context_list[0].client_id >> 0) & 0xFF;
                            tmp.pack_payload[count++] =  0;
                            tmp.pack_payload[count++] = rfnw_calc_crc(tmp.pack_payload + 2, tmp.pack_payload[2] +1);
                            // tmp.pack_len = 14;
                            write(s, tmp.pack_payload, count);

                            rfnw_prepare_pack(&tmp,g_app_context.g_master_id, 0, CMD_READ_CLIENT_ID, NULL, 0);
                            rfnw_task_send_msg(&tmp);
                            break;
                        case SET_ORDER_ID_TO_CLIENT:
                            // <1type> < 4 client id> <1 order id>
                        {
                            uint32_t tmpclid = (uint32_t)payload[1] << 24 | (uint32_t)payload[2] << 16 | (uint32_t)payload[3] << 8 | (uint32_t)payload[4];
                            ESP_LOGI(SERVER_TAG, "Set order id for %x\n", tmpclid);
                            rfnw_prepare_pack(&tmp, g_app_context.g_master_id, tmpclid, CMD_CONFIG_ORDER, payload + 5, 1);
                            rfnw_task_send_msg(&tmp);
                            for(int i = 0; i < MAX_LIGHT; i++){
                                if(g_app_context.light_context_list[i].order_id == payload[5]){
                                    g_app_context.light_context_list[i].client_id = tmpclid;
                                    break;
                                }
                            }
                            uint8_t tmparr[16];
                            tmparr[0] = 0xAA;
                            tmparr[1] = 0x55;
                            tmparr[2] = 6;
                            memcpy(tmparr+ 3, payload, 6);
                            tmparr[3] = SET_ORDER_ID_TO_CLIENT| 0x80;
                            tmparr[9] = rfnw_calc_crc(tmparr+ 2, 7);
                            write(s, tmparr, 10);
                        }
                            break;
                        case CONTROL_BY_ORDER_ID:
                            // <1type> < 4 byte val>
                            {
                                ligh_control_master_out(payload[1]); // control master

                                rfnw_prepare_pack(&tmp,  g_app_context.g_master_id, g_app_context.light_context_list[1].client_id, CMD_CONTROL, payload + 1, 4);
                                rfnw_task_send_msg(&tmp);
                                rfnw_prepare_pack(&tmp,  g_app_context.g_master_id, g_app_context.light_context_list[2].client_id, CMD_CONTROL, payload + 1, 4);
                                rfnw_task_send_msg(&tmp);
                                rfnw_prepare_pack(&tmp,  g_app_context.g_master_id, g_app_context.light_context_list[3].client_id, CMD_CONTROL, payload + 1, 4);
                                rfnw_task_send_msg(&tmp);
                                isControlByApp = true;
                            }

                            break;
                        case SET_CONFIG:
                        {   // fetch data
                            // <1type>< 4 master id> < 4 client id> <1 order id> < 1b out order1><1b time 1>< 1b out order2><1b time 2>< 1b out order3><1b time 3>< 1b out order4><1b time 4>
                            uint32_t tmpclid = (uint32_t)payload[5] << 24 | (uint32_t)payload[6] << 16 | (uint32_t)payload[7] << 8 | (uint32_t)payload[8];
                            ESP_LOGI(SERVER_TAG, "Set config for client id: %x\n", (unsigned int)tmpclid);
                            uint8_t tmporderid = payload[9];
                            for(int i = 0; i < MAX_LIGHT; i++){
                                if(g_app_context.light_context_list[i].order_id == tmporderid){
                                    // g_app_context.light_context_list[i].client_id = tmpclid;
                                    for(int j = 0; j < MAX_OUT; j++){
                                        g_app_context.light_context_list[i].out[j].out_idx = payload[10 + j*2];
                                        g_app_context.light_context_list[i].out[j].interval = payload[11 + j*2];
                                        g_app_context.light_context_list[i].out[j].current_tick = payload[11 + j*2];
                                    }
                                }
                            }
                            uint8_t tmparr[32];
                            tmparr[0] = 0xAA;
                            tmparr[1] = 0x55;
                            tmparr[2] = 1 + 4 + 4 + 1 + MAX_OUT*2; 
                            tmparr[3] = SET_CONFIG | 0x80;
                            for(int i = 0; i < MAX_LIGHT; i++){
                                if(tmporderid == g_app_context.light_context_list[i].order_id) {
                                tmparr[4] = (g_app_context.g_master_id >> 24) & 0xFF;
                                tmparr[5] = (g_app_context.g_master_id >> 16) & 0xFF;
                                tmparr[6] = (g_app_context.g_master_id >> 8) & 0xFF;
                                tmparr[7] = (g_app_context.g_master_id >> 0) & 0xFF;
                                tmparr[8] = (g_app_context.light_context_list[i].client_id >> 24) & 0xFF;
                                tmparr[9] = (g_app_context.light_context_list[i].client_id >> 16) & 0xFF;
                                tmparr[10] = (g_app_context.light_context_list[i].client_id >> 8) & 0xFF;
                                tmparr[11] = (g_app_context.light_context_list[i].client_id >> 0) & 0xFF;
                                tmparr[12] = g_app_context.light_context_list[i].order_id;
                                for(int j = 0; j < MAX_OUT; j++) {
                                    tmparr[13+j*2] = g_app_context.light_context_list[i].out[j].out_idx;
                                    tmparr[14+j*2] = g_app_context.light_context_list[i].out[j].interval;
                                }
                                tmparr[13 + MAX_OUT*2] = rfnw_calc_crc(tmparr+2, tmparr[2] + 1);
                                write(s, tmparr, tmparr[2] + 4);
                                break;
                                }
                            }
                            // call sync task
                            ligh_control_sync_config();
                        }
                        break;
                        case READ_CONFIG:
                        {
                            ESP_LOGI(SERVER_TAG, "Read config \n");
                            uint8_t tmparr[32];
                            tmparr[0] = 0xAA;
                            tmparr[1] = 0x55;
                            tmparr[2] = 1 + 4 + 4 + 1 + MAX_OUT*2; 
                            tmparr[3] = READ_CONFIG | 0x80;
                            for(int i = 0; i < MAX_LIGHT; i++){
                                tmparr[4] = (g_app_context.g_master_id >> 24) & 0xFF;
                                tmparr[5] = (g_app_context.g_master_id >> 16) & 0xFF;
                                tmparr[6] = (g_app_context.g_master_id >> 8) & 0xFF;
                                tmparr[7] = (g_app_context.g_master_id >> 0) & 0xFF;
                                tmparr[8] = (g_app_context.light_context_list[i].client_id >> 24) & 0xFF;
                                tmparr[9] = (g_app_context.light_context_list[i].client_id >> 16) & 0xFF;
                                tmparr[10] = (g_app_context.light_context_list[i].client_id >> 8) & 0xFF;
                                tmparr[11] = (g_app_context.light_context_list[i].client_id >> 0) & 0xFF;
                                tmparr[12] = g_app_context.light_context_list[i].order_id;
                                for(int j = 0; j < MAX_OUT; j++) {
                                    tmparr[13+j*2] = g_app_context.light_context_list[i].out[j].out_idx;
                                    tmparr[14+j*2] = g_app_context.light_context_list[i].out[j].interval;
                                }
                                tmparr[13 + MAX_OUT*2] = rfnw_calc_crc(tmparr+2, tmparr[2] + 1);
                                ESP_LOGI(SERVER_TAG, "Checksum %d \n", tmparr[12 + MAX_OUT*2]);
                                write(s, tmparr, tmparr[2] + 4);
                            }
                        }
                        break;
                        case OUT_CONTROL_CLIENT:
                            {
                                ESP_LOGI(SERVER_TAG, "Out control \n");
                                uint8_t tmparr[8];
                                tmparr[0] = 0xAA;
                                tmparr[1] = 0x55;
                                tmparr[2] = 1;
                                tmparr[3] = OUT_CONTROL_CLIENT | 0x80;
                                tmparr[4] = rfnw_calc_crc(tmparr + 2, 2);
                                write(s, tmparr, 5);
                                isControlByApp = false;
                            }
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

            if(isConnected == false) {
                ESP_LOGI(SERVER_TAG, "Client disconnected client socket %d. CLOSE IT", clientSock);
                close(clientSock);
                break;
            }

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
                        // ESP_LOGI(SERVER_TAG, "Message: %s\n", recv_buf);
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
