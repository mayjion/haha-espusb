// tusb_composite_client.c (Client - Full Implementation)
#include "tusb_composite_main.h"
#include <string.h>
#include <sys/select.h>
#include <fcntl.h>
#include <errno.h>

#define FRAME_BUFFER_SIZE 2048
static uint8_t frame_buffer[FRAME_BUFFER_SIZE];
static uint8_t escape_temp[FRAME_BUFFER_SIZE];

static const char *TAG = "client_main";
static uint8_t cdc_rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
static QueueHandle_t app_queue;
static client_t server_client = { .sock = -1, .active = false, .seq_tx = 1, .seq_rx = 1 };
static bool wifi_connected = false;
static bool tcp_connected = false;
static uint16_t global_seq = 1;  // For logging

// Device descriptor (same as server for compatibility)
static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0xCafe,
    .idProduct = 0x4001,  // Different product ID for client
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

// Configuration descriptor (single CDC)
static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN), TUSB_DESC_CONFIG_ATT_SELF_POWERED, 100),
    TUD_CDC_DESCRIPTOR(0, 4, 0x81, 8, 0x02, 0x82, 64),
};

// String descriptors
static const char *string_desc_arr[] = {
    (const char[]){0x09, 0x04}, // Language: English (US)
    "Manufacturer",
    "ESP32-S3 CDC Client",
    "0987654321",
    "CDC Interface"
};

// Protocol Implementation (copied/adapted from server)
uint16_t compute_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = PROTO_CRC_INIT;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ PROTO_CRC_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void escape_bytes(uint8_t *data, size_t *len) {
    uint8_t temp[FRAME_BUFFER_SIZE];
    size_t orig_len = *len;
    size_t j = 0;
    for (size_t i = 0; i < orig_len; ++i) {
        if (data[i] == 0xFF) {
            temp[j++] = PROTO_ESCAPE;
            temp[j++] = PROTO_ESC_FF;
        } else if (data[i] == PROTO_ESCAPE) {
            temp[j++] = PROTO_ESCAPE;
            temp[j++] = PROTO_ESC_FE;
        } else {
            temp[j++] = data[i];
        }
    }
    memcpy(data, temp, j);
    *len = j;
}

int extract_next_frame(uint8_t *buf, size_t buf_len, uint8_t *type, uint16_t *seq, uint16_t *payload_len, uint8_t *payload, uint16_t *crc) {
    if (buf_len < 7) return 0;

    size_t i = 0;
    while (i + 6 < buf_len) {
        if (buf[i] == 0xFF && buf[i + 1] == 0xFF) {
            *type = buf[i + 2];
            *seq = (buf[i + 3] << 8) | buf[i + 4];
            *payload_len = (buf[i + 5] << 8) | buf[i + 6];

            size_t header_end = i + 7;
            if (header_end > buf_len) {
                // 不完整头部：跳到末尾
                return (int)buf_len;
            }

            // 反转义 payload
            size_t unesc_count = 0;
            size_t raw_pos = header_end;
            size_t j = 0;
            bool parse_fail = false;
            while (raw_pos < buf_len && unesc_count < *payload_len) {
                if (buf[raw_pos] == PROTO_ESCAPE) {
                    if (raw_pos + 1 >= buf_len) {
                        parse_fail = true;
                        break;
                    }
                    uint8_t esc_code = buf[raw_pos + 1];
                    raw_pos += 2;
                    if (esc_code == PROTO_ESC_FF) {
                        payload[j++] = 0xFF;
                    } else if (esc_code == PROTO_ESC_FE) {
                        payload[j++] = PROTO_ESCAPE;
                    } else {
                        parse_fail = true;
                        break;
                    }
                    unesc_count++;
                } else {
                    payload[j++] = buf[raw_pos++];
                    unesc_count++;
                }
            }
            if (parse_fail || unesc_count < *payload_len) {
                // 坏/不完整 payload：估算跳过
                size_t est_skip = 7 + (*payload_len * 2 + 4);  // 最大转义 (x2) + CRC 裕度
                i += est_skip;
                if (i > buf_len) i = buf_len;
                continue;
            }

            // 反转义 CRC
            uint8_t crc_unesc[2];
            unesc_count = 0;
            size_t k = 0;
            parse_fail = false;
            while (raw_pos < buf_len && unesc_count < 2) {
                if (buf[raw_pos] == PROTO_ESCAPE) {
                    if (raw_pos + 1 >= buf_len) {
                        parse_fail = true;
                        break;
                    }
                    uint8_t esc_code = buf[raw_pos + 1];
                    raw_pos += 2;
                    if (esc_code == PROTO_ESC_FF) {
                        crc_unesc[k++] = 0xFF;
                    } else if (esc_code == PROTO_ESC_FE) {
                        crc_unesc[k++] = PROTO_ESCAPE;
                    } else {
                        parse_fail = true;
                        break;
                    }
                    unesc_count++;
                } else {
                    crc_unesc[k++] = buf[raw_pos++];
                    unesc_count++;
                }
            }
            if (parse_fail || unesc_count < 2) {
                // 坏/不完整 CRC：估算跳过
                size_t est_skip = 7 + (*payload_len + 4);  // Payload + CRC 裕度
                i += est_skip;
                if (i > buf_len) i = buf_len;
                continue;
            }

            *crc = (crc_unesc[0] << 8) | crc_unesc[1];
            return (int)(raw_pos - i);  // 消费前缀跳过 + 完整帧（相对 i）
        }
        i++;
    }
    return 0;  // 无帧；等待更多数据
}

bool unescape_bytes(uint8_t *data, size_t *len) {
    uint8_t temp[FRAME_BUFFER_SIZE + 32];
    size_t orig_len = *len;
    size_t j = 0;
    size_t i = 0;
    while (i < orig_len) {
        if (data[i] == PROTO_ESCAPE) {
            if (i + 1 < orig_len) {
                ++i;
                if (data[i] == PROTO_ESC_FF) {
                    temp[j++] = 0xFF;
                } else if (data[i] == PROTO_ESC_FE) {
                    temp[j++] = PROTO_ESCAPE;
                } else {
                    return false;
                }
                ++i;
            } else {
                return false;
            }
        } else {
            temp[j++] = data[i++];
        }
        if (j >= sizeof(temp)) return false;
    }
    memcpy(data, temp, j);
    *len = j;
    return true;
}

bool parse_frame(const uint8_t *buf, size_t len, uint8_t *type, uint16_t *seq, uint16_t *payload_len, uint8_t *payload, uint16_t *crc) {
    if (len < 7) return false;
    if (buf[0] != 0xFF || buf[1] != 0xFF) return false;
    *type = buf[2];
    *seq = (buf[3] << 8) | buf[4];
    *payload_len = (buf[5] << 8) | buf[6];
    if (len < 7 + *payload_len + 2) return false;
    memcpy(payload, &buf[7], *payload_len);
    *crc = (buf[7 + *payload_len] << 8) | buf[7 + *payload_len + 1];
    uint16_t calc_crc = compute_crc16(&buf[2], 1 + 2 + 2 + *payload_len);
    return calc_crc == *crc;
}

// 更新build_frame：返回帧长度，不直接转义
size_t build_frame(uint8_t *buf, uint8_t type, uint16_t seq, const uint8_t *payload, uint16_t payload_len, uint16_t *out_crc) {
    // 头部固定，不转义
    buf[0] = 0xFF; buf[1] = 0xFF;
    buf[2] = type;
    buf[3] = seq >> 8; buf[4] = seq & 0xFF;
    buf[5] = payload_len >> 8; buf[6] = payload_len & 0xFF;
    
    // 复制payload（未转义）
    if (payload_len > 0) {
        memcpy(&buf[7], payload, payload_len);
    }
    
    // 计算CRC（基于type + seq + len + payload，未转义）
    *out_crc = compute_crc16(&buf[2], 1 + 2 + 2 + payload_len);
    
    // 放置CRC（未转义）
    size_t crc_offset = 7 + payload_len;
    buf[crc_offset] = *out_crc >> 8;
    buf[crc_offset + 1] = *out_crc & 0xFF;
    
    return 7 + payload_len + 2;  // 返回未转义帧长度
}

// 更新send_reliable：只转义payload + CRC部分
esp_err_t send_reliable(client_t *client, uint8_t type, const uint8_t *payload, uint16_t payload_len) {
    uint8_t frame[FRAME_BUFFER_SIZE];
    uint16_t crc;
    size_t raw_len = build_frame(frame, type, client->seq_tx, payload, payload_len, &crc);
    
    // 只转义payload + CRC（从offset 7开始）
    size_t esc_part_len = payload_len + 2;  // payload + CRC
    escape_bytes(&frame[7], &esc_part_len);
    
    size_t frame_len = 7 + esc_part_len;  // 头部+类型+seq+len (未转义) + 转义后的payload+CRC
    
    ESP_LOGD(TAG, "Built frame len=%zu (raw=%zu, esc_part=%zu)", frame_len, raw_len, esc_part_len);
    
    // 发送重试
    for (int retry = 0; retry < PROTO_MAX_RETRIES; retry++) {
        int sent = send(client->sock, frame, frame_len, 0);
        if (sent == (int)frame_len) {
            client->seq_tx = (client->seq_tx + 1) % PROTO_SEQ_MAX;
            ESP_LOGD(TAG, "Sent frame seq=%d, type=0x%02x, len=%d", client->seq_tx - 1, type, payload_len);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "Send retry %d/%d: sent=%d, expected=%zu", retry + 1, PROTO_MAX_RETRIES, sent, frame_len);
        vTaskDelay(pdMS_TO_TICKS(PROTO_TIMEOUT_MS * (retry + 1)));
    }
    return ESP_FAIL;
}
bool handle_response(client_t *client, uint8_t *payload, uint16_t payload_len) {
    if (payload_len < 2) return false;
    uint8_t tag = payload[0];
    uint8_t result = payload[1];
    if (result == 0x00) {  // ACK
        return true;
    }
    ESP_LOGW(TAG, "NAK received: %02x", result);
    return false;
}

static void send_to_server(const uint8_t *data, size_t len) {
    if (!tcp_connected) {
        ESP_LOGW(TAG, "TCP not connected, dropping data");
        return;
    }
    esp_err_t ret = send_reliable(&server_client, PROTO_TYPE_DATA, data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send reliable failed, reconnecting");
        tcp_connected = false;
        close(server_client.sock);
        server_client.sock = -1;
    }
}

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event) {
    size_t rx_size = 0;
    esp_err_t ret = tinyusb_cdcacm_read(itf, cdc_rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK && rx_size > 0) {
        app_message_t msg = { .buf_len = rx_size, .itf = itf };
        memcpy(msg.buf, cdc_rx_buf, rx_size);
        if (xQueueSend(app_queue, &msg, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Queue full, packet dropped");
        }
    }
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event) {
    if (!event->line_state_changed_data.dtr || !event->line_state_changed_data.rts) {
        tud_cdc_n_write_clear(TINYUSB_CDC_ACM_0);
        tud_cdc_n_read_flush(TINYUSB_CDC_ACM_0);
    }
}

void tinyusb_cdc_line_coding_changed_callback(int itf, cdcacm_event_t *event) {
    cdcacm_event_line_coding_changed_data_t *coding = &event->line_coding_changed_data;
    char message[128];
    snprintf(message, sizeof(message), "BAUD=%lu,STOPBIT=%d,PARITY=%d,DATABIT=%d\r\n",
             coding->p_line_coding->bit_rate, coding->p_line_coding->stop_bits,
             coding->p_line_coding->parity, coding->p_line_coding->data_bits);
    ESP_LOGI(TAG, "Baud rate update: %s", message);
    // No send, as client
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi disconnected, retrying");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
    }
}

void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = { };
    strlcpy((char*)wifi_config.sta.ssid, SERVER_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char*)wifi_config.sta.password, SERVER_PASS, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA started, connecting to %s", SERVER_SSID);
}

esp_err_t tcp_client_connect(void) {
    if (server_client.sock >= 0) {
        close(server_client.sock);
        server_client.sock = -1;
    }

    server_client.sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_client.sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return ESP_FAIL;
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    int ret = connect(server_client.sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (ret != 0) {
        ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
        close(server_client.sock);
        server_client.sock = -1;
        return ESP_FAIL;
    }

    server_client.active = true;
    server_client.last_heartbeat = xTaskGetTickCount() * portTICK_PERIOD_MS;
    tcp_connected = true;
    ESP_LOGI(TAG, "TCP connected to %s:%d", SERVER_IP, PORT);
    return ESP_OK;
}

static void tcp_client_task(void *pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);

    // Wait for WiFi
    while (!wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    while (1) {
        esp_task_wdt_reset();
        if (!tcp_connected) {
            if (tcp_client_connect() != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
        }

        uint8_t rx_buf[RX_BUFFER_SIZE];
        size_t rx_len = 0;
        fd_set readfds;
        struct timeval timeout = {1, 0};
        FD_ZERO(&readfds);
        FD_SET(server_client.sock, &readfds);
        int sel_ret = select(server_client.sock + 1, &readfds, NULL, NULL, &timeout);
        if (sel_ret <= 0) {
            // Heartbeat check
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - server_client.last_heartbeat > PROTO_HEARTBEAT_INTERVAL) {
                send_reliable(&server_client, PROTO_TYPE_HEARTBEAT, NULL, 0);
                server_client.last_heartbeat = now;
            }
            if (sel_ret < 0) {
                ESP_LOGW(TAG, "Select error: %d", errno);
                tcp_connected = false;
                continue;
            }
            continue;
        }

        int ret = recv(server_client.sock, rx_buf + rx_len, sizeof(rx_buf) - rx_len, 0);
        if (ret <= 0) {
            ESP_LOGW(TAG, "TCP recv error/close: %d", errno);
            tcp_connected = false;
            close(server_client.sock);
            server_client.sock = -1;
            server_client.active = false;
            continue;
        }
        rx_len += ret;
        ESP_LOGD(TAG, "Recv from server: %d bytes, total=%zu", ret, rx_len);

        // Process frames
        uint8_t type, payload[512];
        uint16_t seq, plen, crc_val;
        size_t processed = 0;
        while (rx_len - processed >= 7) {
            int frame_len = extract_next_frame(rx_buf + processed, rx_len - processed, &type, &seq, &plen, payload, &crc_val);
            if (frame_len <= 0) {
                // 不完整/坏：跳过 1 字节防循环
                processed += 1;
                continue;
            }

            // 内联 CRC 验证：重构未转义 header + payload
            uint8_t crc_input[1 + 2 + 2 + 512];
            crc_input[0] = type;
            crc_input[1] = (seq >> 8) & 0xFF;
            crc_input[2] = seq & 0xFF;
            crc_input[3] = (plen >> 8) & 0xFF;
            crc_input[4] = plen & 0xFF;
            if (plen > 512) {
                ESP_LOGE(TAG, "Payload overflow: %d > 512", plen);
                processed += frame_len;
                continue;
            }
            memcpy(&crc_input[5], payload, plen);  // 未转义 payload
            uint16_t calc_crc = compute_crc16(crc_input, 5 + plen);

            if (calc_crc != crc_val) {
                ESP_LOGW(TAG, "CRC mismatch: calc=0x%04x != recv=0x%04x (type=0x%02x seq=%d plen=%d)",
                         calc_crc, crc_val, type, seq, plen);
                processed += frame_len;
                continue;
            }

            // Seq check
            bool seq_match = (seq == server_client.seq_rx);
            if (!seq_match) {
                ESP_LOGW(TAG, "Seq mismatch: expected %d, got %d (type=0x%02x)", server_client.seq_rx, seq, type);
                uint8_t nak[2] = {0x00, 0x01};
                send_reliable(&server_client, PROTO_TYPE_RESP, nak, 2);
                processed += frame_len;
                continue;
            }

            // Handle by type (仅成功后递增 seq_rx)
            bool handled = false;
            if (type == PROTO_TYPE_DATA) {
                ESP_LOGI(TAG, "Received DATA seq=%d len=%d from server", seq, plen);
                // Send to USB CDC
                size_t queued = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, payload, plen);
                esp_err_t flush_ret = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(CDC_FLUSH_TIMEOUT_MS));
                if (queued == plen && flush_ret == ESP_OK) {
                    ESP_LOGD(TAG, "USB write success: %zu bytes", plen);
                    handled = true;
                } else {
                    ESP_LOGW(TAG, "USB write fail: queued=%zu/%d, flush=%s", queued, plen, esp_err_to_name(flush_ret));
                    // USB 失败不递增 seq_rx（允许重发）
                }
                uint8_t ack[2] = {0x00, 0x00};
                send_reliable(&server_client, PROTO_TYPE_RESP, ack, 2);
            } else if (type == PROTO_TYPE_RESP) {
                if (handle_response(&server_client, payload, plen)) {
                    ESP_LOGD(TAG, "Handled ACK RESP seq=%d", seq);
                    handled = true;
                } else {
                    ESP_LOGW(TAG, "Bad RESP seq=%d", seq);
                    // 可选：如果 seq 匹配 last_tx，重发最后 DATA
                }
            } else if (type == PROTO_TYPE_HEARTBEAT) {
                server_client.last_heartbeat = xTaskGetTickCount() * portTICK_PERIOD_MS;
                send_reliable(&server_client, PROTO_TYPE_HEARTBEAT, NULL, 0);
                ESP_LOGI(TAG, "HB exchanged seq=%d", seq);
                handled = true;
            }
            if (handled) {
                server_client.seq_rx = (server_client.seq_rx + 1) % PROTO_SEQ_MAX;
            }

            processed += frame_len;
        }

        // Shift buffer
        if (processed > 0) {
            rx_len -= processed;
            if (rx_len > 0) {
                memmove(rx_buf, rx_buf + processed, rx_len);
            }
        }
        ESP_LOGD(TAG, "Processed %zu bytes, remaining %zu", processed, rx_len);
    }
    esp_task_wdt_delete(NULL);
    vTaskDelete(NULL);
}

static void button_task(void *pvParameters) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    TickType_t last_press_time = 0;
    bool last_state = true;
    while (1) {
        bool current_state = gpio_get_level(BUTTON_GPIO);
        if (last_state && !current_state) {
            last_press_time = xTaskGetTickCount();
        } else if (!last_state && current_state) {
            TickType_t press_duration = xTaskGetTickCount() - last_press_time;
            if (pdTICKS_TO_MS(press_duration) > BUTTON_LONG_PRESS_MS) {
                // Clear NVS if needed, but client has no NVS config
                esp_restart();
            }
        }
        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
    }
    vTaskDelete(NULL);
}

static void button_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void led_task(void *pvParameters) {
    ESP_LOGI(TAG, "LED task started");
    while (1) {
        if (!wifi_connected) {
            // Slow flash: 500ms on/off
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else if (!tcp_connected) {
            // Fast flash: 100ms on/off
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            // Solid on
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static esp_err_t init_system(void) {
    // LED init
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(LED_GPIO, 0);

    // NVS for WiFi? Not needed, but init anyway
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    wifi_init_sta();
    button_init();

    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("tusb", ESP_LOG_DEBUG);

    app_queue = xQueueCreate(QUEUE_SIZE, sizeof(app_message_t));
    if (!app_queue) {
        ESP_LOGE(TAG, "Failed to create app_queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t init_usb(void) {
    tinyusb_config_t tusb_cfg = {
        .device_descriptor = &desc_device,
        .string_descriptor = string_desc_arr,
        .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
        .external_phy = false,
        .configuration_descriptor = desc_configuration
    };
    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSB install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = CONFIG_TINYUSB_CDC_RX_BUFSIZE,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_line_state_changed = &tinyusb_cdc_line_state_changed_callback,
        .callback_line_coding_changed = &tinyusb_cdc_line_coding_changed_callback,
    };
    ret = tusb_cdc_acm_init(&acm_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CDC ACM init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "USB CDC initialized successfully");
    return ESP_OK;
}

static void usb_device_task(void *param) {
    vTaskDelay(pdMS_TO_TICKS(100));
    while (1) {
        tud_task();
        vTaskDelay(1);
    }
}

static void usb_to_tcp_task(void *param) {
    app_message_t msg;
    while (1) {
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Raw CDC data received (len=%zu):", msg.buf_len);
            ESP_LOG_BUFFER_HEX(TAG, msg.buf, msg.buf_len);
            send_to_server(msg.buf, msg.buf_len);
        }
    }
}

void app_main(void) {
    esp_err_t ret = init_system();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "System init failed, restarting");
        esp_restart();
    }
    ret = init_usb();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB init failed, restarting");
        esp_restart();
    }

    xTaskCreate(usb_device_task, "usb_dev", 49152, NULL, 10, NULL);
    xTaskCreate(tcp_client_task, "tcp_client", 16384, NULL, 8, NULL);
    xTaskCreate(usb_to_tcp_task, "usb_to_tcp", 8192, NULL, 9, NULL);
    xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 3, NULL);
    ESP_LOGI(TAG, "Tasks created, app_main complete");
}