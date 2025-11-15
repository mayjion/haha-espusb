#include "tcp_client.h"
#include "protocol.h"  // For frame building/parsing
#include "usbhandle.h"  // For tx_ring and app_queue
#include <string.h>    // For memset, memcpy
#include <sys/socket.h> // For send/recv

static const char *TAG = "tcpclient";
client_t active_client = {0};
SemaphoreHandle_t client_mutex = NULL;  // Added: For thread-safe access to active_client
tx_ring_t tx_ring = {0};  // Moved: Shared with USB, initialized here or in usbhandle
QueueHandle_t app_queue;  // Moved: Shared with USB
rx_ring_t rx_ring = {0};  // New
SemaphoreHandle_t rx_mutex = NULL;  // New

void wdt_init(uint32_t timeout_ms) {  // CHANGED: uint8_t -> uint32_t
    esp_err_t wdt_deinit = esp_task_wdt_deinit();
    if (wdt_deinit == ESP_ERR_NOT_FOUND) {
        ESP_LOGD(TAG, "TWDT was not initialized, proceeding to init");
    } else if (wdt_deinit != ESP_OK) {
        ESP_LOGW(TAG, "TWDT deinit failed: %s", esp_err_to_name(wdt_deinit));
    } else {
        ESP_LOGI(TAG, "TWDT deinit success");
    }
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = timeout_ms,  // Now safe for 30000
        .idle_core_mask = 0,  // Monitor all cores
        .trigger_panic = false,
    };
    esp_err_t wdt_err = esp_task_wdt_init(&wdt_config);
    if (wdt_err != ESP_OK) {
        ESP_LOGW(TAG, "TWDT init failed: %s", esp_err_to_name(wdt_err));
    } else {
        ESP_LOGI(TAG, "TWDT initialized with %ums timeout", (unsigned)timeout_ms);  // Cast for log
    }
}

void client_init(void) { 
    memset(&active_client, 0, sizeof(active_client));
    active_client.sock = -1;
    active_client.active = false;
    active_client.seq_tx = 1;
    active_client.seq_rx = 1;
}

esp_err_t init_system(void) {
    // GPIO init
    led_init();
    button_init();

    // WiFi init (now includes full stack: NVS + netif + event loop)
    wifi_config_init();  // ADDED: Full init (replaces manual nvs_flash_init)
    wifi_init_sta();     // Start STA connection

    // WDT init
    wdt_init(30000);  // Will be fixed in #2

    // Mutex and shared resources
    client_mutex = xSemaphoreCreateMutex();
    if (client_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create client_mutex");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "client_mutex created OK");  // 添加：确认创建成功
    // Ring buffer init: Dynamic heap alloc to avoid .bss issues
    // TX ring
    tx_ring.buf = calloc(TX_BUFFER_SIZE, sizeof(uint8_t));  // Alloc + zero
    if (tx_ring.buf == NULL) {
        ESP_LOGE(TAG, "Failed to alloc tx_ring.buf (%d bytes)", TX_BUFFER_SIZE);
        return ESP_FAIL;
    }
    tx_ring.head = 0;
    tx_ring.tail = 0;
    tx_ring.len = 0;
    tx_ring.size = TX_BUFFER_SIZE;
    ESP_LOGI(TAG, "TX ring allocated and zeroed (%zu bytes)", TX_BUFFER_SIZE);

    // RX ring
    rx_ring.buf = calloc(RX_RING_SIZE, sizeof(uint8_t));  // Alloc + zero
    if (rx_ring.buf == NULL) {
        ESP_LOGE(TAG, "Failed to alloc rx_ring.buf (%d bytes)", RX_RING_SIZE);
        free(tx_ring.buf);  // Cleanup
        return ESP_FAIL;
    }
    rx_ring.head = 0;
    rx_ring.tail = 0;
    rx_ring.len = 0;
    rx_ring.size = RX_RING_SIZE;
    ESP_LOGI(TAG, "RX ring allocated and zeroed (%zu bytes)", RX_RING_SIZE);

    rx_mutex = xSemaphoreCreateMutex();
    if (rx_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create rx_mutex");
        free(tx_ring.buf);
        free(rx_ring.buf);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t rx_ring_append(const uint8_t *data, size_t len) {
    if (xSemaphoreTake(rx_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return ESP_ERR_TIMEOUT;
    while (len > 0) {
        size_t chunk = (len > 512) ? 512 : len;
        // Overflow check/drop here (per chunk)
        if (rx_ring.len + chunk > rx_ring.size) {
            size_t drop = rx_ring.len + chunk - rx_ring.size;
            rx_ring.tail = (rx_ring.tail + drop) % rx_ring.size;
            rx_ring.len -= drop;
            ESP_LOGW(TAG, "RX overflow, dropped %zu", drop);
        }
        for (size_t i = 0; i < chunk; i++) {
            rx_ring.buf[rx_ring.head] = data[i];
            rx_ring.head = (rx_ring.head + 1) % rx_ring.size;
        }
        rx_ring.len += chunk;
        data += chunk; len -= chunk;
        if (len > 0) xSemaphoreGive(rx_mutex), xSemaphoreTake(rx_mutex, pdMS_TO_TICKS(1));  // Quick re-take
    }
    xSemaphoreGive(rx_mutex);
    return ESP_OK;
}


// New: Consume from RX ring (up to max_len, returns consumed)
size_t rx_ring_consume(uint8_t *buf, size_t max_len) {
    if (xSemaphoreTake(rx_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {  // Increased to 100ms
        ESP_LOGI("tcpclient", "RX mutex take timeout in consume (contention)");  // New: Diagnose
        return 0;
    }
    size_t consumed = (rx_ring.len < max_len) ? rx_ring.len : max_len;
    for (size_t i = 0; i < consumed; i++) {
        buf[i] = rx_ring.buf[rx_ring.tail];  // Unchanged
        rx_ring.tail = (rx_ring.tail + 1) % rx_ring.size;  // CHANGED: % .size
    }
    rx_ring.len -= consumed;
    xSemaphoreGive(rx_mutex);
    return consumed;
}


// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi disconnected, retrying connect");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void tcp_client_task(void *pvParameters) {
    esp_task_wdt_add(NULL);
    client_init();

    // Register WiFi event handler for auto-reconnect
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    while (1) {
        // Wait for WiFi connected (poll every 5s)
        wifi_ap_record_t ap_info;
        esp_err_t wifi_ret = esp_wifi_sta_get_ap_info(&ap_info);
        if (wifi_ret != ESP_OK) {
            ESP_LOGI(TAG, "WiFi not connected, retrying in 5s");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        ESP_LOGI(TAG, "WiFi connected to %s, attempting TCP connect", ap_info.ssid);

        // Create socket
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // Set non-blocking
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);

        // Server addr
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        active_client.addr = dest_addr;

        // Connect with timeout
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0 && errno != EINPROGRESS) {
            ESP_LOGE(TAG, "Socket connect failed: errno %d", errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // Poll for connect
        struct timeval timeout = { .tv_sec = 10, .tv_usec = 0 };
        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(sock, &writefds);
        err = select(sock + 1, NULL, &writefds, NULL, &timeout);
        if (err < 0) {
            ESP_LOGE(TAG, "Select failed: errno %d", errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        if (!FD_ISSET(sock, &writefds)) {
            ESP_LOGW(TAG, "Connect timeout");
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // Connected
        xSemaphoreTake(client_mutex, portMAX_DELAY);
        active_client.sock = sock;
        active_client.active = true;
        xSemaphoreGive(client_mutex);
        ESP_LOGI(TAG, "TCP connected to %s:%d", SERVER_IP, PORT);

        // RX loop
        uint8_t rx_buf[RX_BUFFER_SIZE];
        while (active_client.active) {
            int rx_len = recv(sock, rx_buf, sizeof(rx_buf), MSG_DONTWAIT);
            if (rx_len > 0) {
                ESP_LOGD(TAG, "Received %d bytes from server", rx_len);
                rx_ring_append(rx_buf, rx_len);
            } else if (rx_len == 0) {
                ESP_LOGW(TAG, "Server closed connection");
                break;
            } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGE(TAG, "Recv error: %d", errno);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            esp_task_wdt_reset();
        }

        // Cleanup
        xSemaphoreTake(client_mutex, portMAX_DELAY);
        active_client.active = false;
        shutdown(sock, 0);
        close(sock);
        active_client.sock = -1;
        xSemaphoreGive(client_mutex);
        ESP_LOGI(TAG, "TCP disconnected, retrying in 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void send_task(void *pvParameters) {
    esp_task_wdt_add(NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1));  // Keep for low latency, but consider 10ms idle.

        xSemaphoreTake(client_mutex, portMAX_DELAY);
        if (!active_client.active || tx_ring.len == 0) {
            xSemaphoreGive(client_mutex);
            esp_task_wdt_reset();  // ADDED: Reset on idle loop (covers no-client case).
            continue;
        }

        // Improved chunk size logic
        size_t chunk_size = tx_ring.len;
        if (chunk_size > 512) {
            chunk_size = 512;
        } else if (chunk_size > MAX_TX_SIZE) {
            chunk_size = MAX_TX_SIZE;
        }
        ESP_LOGD(TAG, "Sending chunk_size=%zu (ring len=%zu, tail=%zu)", chunk_size, tx_ring.len, tx_ring.tail);

        // Fixed copy loop: Use for loop for exact chunk_size bytes
        size_t pos = tx_ring.tail;
        uint8_t temp_payload[MAX_TX_SIZE];
        memset(temp_payload, 0, sizeof(temp_payload));  // 必须添加：清除栈垃圾
        // ESP_LOGI(TAG, "Starting copy loop: chunk_size=%zu", chunk_size);  // 调试：确认 chunk_size
        for (size_t i = 0; i < chunk_size; ++i) {
            temp_payload[i] = tx_ring.buf[pos];
            pos = (pos + 1) % tx_ring.size;  // 修改：使用 tx_ring.size 而非 TX_BUFFER_SIZE（一致性）
            // if (i < 5) {  // 调试：只日志前5个迭代，确认循环执行
            //     ESP_LOGI(TAG, "Copy i=%zu: temp[%zu]=0x%02X from buf[%zu]=0x%02X", i, i, temp_payload[i], (tx_ring.tail + i) % tx_ring.size, tx_ring.buf[(tx_ring.tail + i) % tx_ring.size]);
            // }
        }
        // ESP_LOGI(TAG, "Copy loop complete: final pos=%zu", pos);  // 调试：确认循环结束
        size_t copied = chunk_size;
        ESP_LOGI(TAG, "TCP TX: %u bytes", copied);
        // ESP_LOG_BUFFER_HEX(TAG, temp_payload, copied);  

        // Set pending for this chunk
        active_client.pending_type = PROTO_TYPE_DATA;
        active_client.pending_seq = active_client.seq_tx;
        active_client.pending_len = copied;
        memcpy(active_client.pending_payload, temp_payload, copied);
        active_client.pending = true;
        xSemaphoreGive(client_mutex);

        // Send with reliability
        esp_err_t ret = send_reliable(&active_client, PROTO_TYPE_DATA, temp_payload, copied);
        xSemaphoreTake(client_mutex, portMAX_DELAY);
        if (ret == ESP_OK) {
            // Success: advance ring
            tx_ring.tail = pos;
            tx_ring.len -= copied;
            active_client.seq_tx++;
            ESP_LOGD(TAG, "Sent chunk seq=%d len=%zu (ring len now=%zu)", active_client.pending_seq, copied, tx_ring.len);
        }
        active_client.pending = false;
        xSemaphoreGive(client_mutex);
        
        esp_task_wdt_reset();  // ADDED: Reset post-send (covers active path).
    }
}

esp_err_t send_control_to_tcp(uint8_t type, const uint8_t *payload, uint16_t payload_len) {
    xSemaphoreTake(client_mutex, pdMS_TO_TICKS(100));
    if (!active_client.active || active_client.sock < 0) {
        xSemaphoreGive(client_mutex);
        ESP_LOGD(TAG, "send_control_to_tcp: No active client, skipping type=0x%02X", type);
        return ESP_FAIL;
    }

    // 堆分配缓冲：frame (总帧), escaped_payload (转义后负载), crc_data (CRC 计算用)
    uint8_t *frame = malloc(FRAME_BUFFER_SIZE);  // ~2.3KB
    uint8_t *escaped_payload = malloc(MAX_TX_SIZE * 2);  // ~2KB
    uint8_t *crc_data = malloc(5 + MAX_TX_SIZE);  // ~1KB
    if (!frame || !escaped_payload || !crc_data) {
        ESP_LOGE(TAG, "Malloc failed for control frame buffers (type=0x%02X, len=%u)", type, payload_len);
        if (frame) free(frame);
        if (escaped_payload) free(escaped_payload);
        if (crc_data) free(crc_data);
        xSemaphoreGive(client_mutex);
        return ESP_FAIL;
    }

    // 清零缓冲
    memset(frame, 0, FRAME_BUFFER_SIZE);
    memset(escaped_payload, 0, MAX_TX_SIZE * 2);
    memset(crc_data, 0, 5 + MAX_TX_SIZE);

    // Build header (unescaped len)
    frame[0] = 0xFF;
    frame[1] = 0xFF;
    frame[2] = type;
    frame[3] = (0 >> 8) & 0xFF;  // seq=0 for control
    frame[4] = 0 & 0xFF;
    frame[5] = (payload_len >> 8) & 0xFF;
    frame[6] = payload_len & 0xFF;

    // Compute CRC over unescaped: header fields + original payload
    uint16_t seq = 0;
    crc_data[0] = type;
    crc_data[1] = (seq >> 8) & 0xFF;
    crc_data[2] = seq & 0xFF;
    crc_data[3] = (payload_len >> 8) & 0xFF;
    crc_data[4] = payload_len & 0xFF;
    memcpy(crc_data + 5, payload, payload_len);
    uint16_t crc = compute_crc16(crc_data, 5 + payload_len);

    // Escape payload for TX
    memcpy(escaped_payload, payload, payload_len);
    uint16_t esc_len = payload_len;
    escape_bytes(escaped_payload, &esc_len);

    // Append escaped payload
    memcpy(&frame[7], escaped_payload, esc_len);

    // Escape and append CRC (修复：使用独立缓冲避免溢出)
    uint8_t temp_crc[2] = {(crc >> 8) & 0xFF, crc & 0xFF};
    uint8_t temp_crc_esc[4];  // 足够大缓冲 (max 4 字节 escaped)
    memcpy(temp_crc_esc, temp_crc, 2);
    uint16_t crc_esc_len = 2;
    escape_bytes(temp_crc_esc, &crc_esc_len);
    memcpy(&frame[7 + esc_len], temp_crc_esc, crc_esc_len);

    size_t frame_len = 7 + esc_len + crc_esc_len;

    // 日志：确认构建
    ESP_LOGI(TAG, "Built control frame: type=0x%02X, seq=%u, payload_len=%u, crc=0x%04X, total_frame=%zu bytes", 
             type, seq, payload_len, crc, frame_len);

    // 发送（带重试）
    esp_err_t ret = ESP_FAIL;
    for (int retry = 0; retry < PROTO_MAX_RETRIES; retry++) {
        int sent = send(active_client.sock, frame, frame_len, 0);
        if (sent == (int)frame_len) {
            ret = ESP_OK;
            ESP_LOGI(TAG, "Sent formatted control msg type=0x%02X len=%u (frame %zu bytes)", type, payload_len, frame_len);
            break;
        }
        ESP_LOGW(TAG, "Control send retry %d/%d: sent=%d expected=%zu", retry + 1, PROTO_MAX_RETRIES, sent, frame_len);
        vTaskDelay(pdMS_TO_TICKS(PROTO_TIMEOUT_MS * (retry + 1)));
    }

    // 释放堆缓冲
    free(frame);
    free(escaped_payload);
    free(crc_data);

    xSemaphoreGive(client_mutex);
    return ret;
}


esp_err_t send_reliable(client_t *client, uint8_t type, const uint8_t *payload, uint16_t payload_len) {
    if (type != PROTO_TYPE_DATA) {
        // For CMD, HB, RESP, etc.: seq fixed to 0, use retry (no pending buffer)
        uint16_t seq = 0;
        uint8_t frame[FRAME_BUFFER_SIZE];
        size_t frame_len = build_escaped_frame(frame, type, seq, payload, payload_len, NULL);
        
        ESP_LOGD(TAG, "Built non-DATA frame len=%zu seq=0 type=0x%02x", frame_len, type);
        
        // Retry loop with timeout
        for (int retry = 0; retry < PROTO_MAX_RETRIES; retry++) {
            int sent = send(client->sock, frame, frame_len, 0);
            if (sent == (int)frame_len) {
                ESP_LOGD(TAG, "Sent non-DATA frame seq=0 type=0x%02x len=%d", type, payload_len);
                return ESP_OK;
            }
            ESP_LOGW(TAG, "Non-DATA send retry %d/%d: sent=%d expected=%zu", retry + 1, PROTO_MAX_RETRIES, sent, frame_len);
            vTaskDelay(pdMS_TO_TICKS(PROTO_TIMEOUT_MS * (retry + 1)));
        }
        ESP_LOGW(TAG, "Non-DATA send failed after %d retries type=0x%02x", PROTO_MAX_RETRIES, type);
        return ESP_FAIL;
    } else {
        // For DATA: assume pending already set by caller (send_task), just send the frame
        uint8_t frame[FRAME_BUFFER_SIZE];
        size_t frame_len = build_escaped_frame(frame, type, client->pending_seq, payload, payload_len, NULL);
        
        // ADDED: Non-blocking send with partial retry and timeout
        TickType_t send_start = xTaskGetTickCount();
        int max_delay_ms = 5000;  // 5s max per frame
        int loop_count = 0;
        const int max_loops = 5000;  // Fallback ~5s at 1ms
        
        int total_sent = 0;
        while (total_sent < (int)frame_len) {
            int sent = send(client->sock, &frame[total_sent], frame_len - total_sent, MSG_DONTWAIT | MSG_NOSIGNAL);
            if (sent <= 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    vTaskDelay(pdMS_TO_TICKS(1));  // Yield 1ms
                    loop_count++;
                    if ((xTaskGetTickCount() - send_start > pdMS_TO_TICKS(max_delay_ms)) ||
                        (loop_count > max_loops)) {
                        ESP_LOGW(TAG, "Send stalled after %d loops/~%dms, dropping seq=%d remain=%d",
                                 loop_count, (xTaskGetTickCount() - send_start) * portTICK_PERIOD_MS,
                                 client->pending_seq, frame_len - total_sent);
                        return ESP_FAIL;
                    }
                    continue;
                }
                ESP_LOGW(TAG, "DATA send fail: %d (errno %d) seq=%d", sent, errno, client->pending_seq);
                return ESP_FAIL;
            }
            total_sent += sent;
            loop_count = 0;  // Reset on progress
            send_start = xTaskGetTickCount();  // Refresh timer
        }
        // ESP_LOG_BUFFER_HEX(TAG, frame, frame_len);

        ESP_LOGD(TAG, "Sent DATA seq=%d len=%d (full %zu bytes)", client->pending_seq, payload_len, frame_len);
        return ESP_OK;
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


    xTaskCreatePinnedToCore(tcp_client_task, "tcp_client", 16384, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(usb_to_tcp_task, "usb_to_tcp", 12288, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(send_task, "send_task", 12288, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(parse_and_usb_task, "parse_usb", 16384, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(button_task, "button_task", 4096, NULL, 2, NULL, 0);  // Core 0 for GPIO
    xTaskCreatePinnedToCore(led_task, "led_task", 4096, NULL, 1, NULL, 0);
    ESP_LOGI(TAG, "Tasks created, app_main complete");
    
}