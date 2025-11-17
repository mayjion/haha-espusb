#include "tcp_server.h"
#include "protocol.h"  // Added: For frame building/parsing
#include "usbhandle.h"  // Added: For tx_ring and app_queue
#include <string.h>    // For memset, memcpy
#include <sys/socket.h> // For send/recv

static const char *TAG = "tcpserver";
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

    // Load and start WiFi
    load_wifi_credentials();
    wifi_init_softap();

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
        ESP_LOGI("tcpserver", "RX mutex take timeout in consume (contention)");  // New: Diagnose
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


void tcp_server_task(void *pvParameters) {
    esp_task_wdt_add(NULL);
    esp_task_wdt_reset(); // Early reset
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket created");
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in dest_addr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_family = AF_INET,
        .sin_port = htons(PORT)
    };
    if (bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);
    if (listen(sock, 1) != 0) {
        ESP_LOGE(TAG, "Error during listen: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    const TickType_t loop_delay = pdMS_TO_TICKS(50); // 50ms outer loop for responsiveness
    struct timeval accept_to = {0, 10000}; // Reduced to 10ms for better responsiveness
    while (1) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(sock, &readfds);
        int sel_ret = select(sock + 1, &readfds, NULL, NULL, &accept_to);
        if (sel_ret < 0) {
            ESP_LOGE(TAG, "Accept select error: %d", errno);
            esp_task_wdt_reset();
            vTaskDelay(loop_delay);
            continue;
        } else if (sel_ret > 0 && FD_ISSET(sock, &readfds)) {
            struct sockaddr_in source_addr;
            socklen_t addr_len = sizeof(source_addr);
            int client_sock = accept(sock, (struct sockaddr *)&source_addr, &addr_len);
            if (client_sock < 0) {
                ESP_LOGE(TAG, "Accept failed: errno %d", errno);
                continue;
            }
            ESP_LOGI(TAG, "New client connected from " IPSTR ":%d",
                     IP2STR((ip4_addr_t *)&source_addr.sin_addr), ntohs(source_addr.sin_port));
            xSemaphoreTake(client_mutex, portMAX_DELAY);
            active_client.sock = client_sock;
            active_client.active = true;
            active_client.addr = source_addr;
            active_client.seq_tx = 1;
            active_client.seq_rx = 1;
            active_client.last_heartbeat = xTaskGetTickCount();
            active_client.pending = false;
            xSemaphoreGive(client_mutex);
            // Set RX buffer size
            int rx_buf_size = 64 * 1024; // 64KB for high-throughput
            setsockopt(client_sock, SOL_SOCKET, SO_RCVBUF, &rx_buf_size, sizeof(rx_buf_size));
            ESP_LOGI(TAG, "Set client RX buf to %d bytes", rx_buf_size);
            // Set non-blocking mode for client_sock
            int flags = fcntl(client_sock, F_GETFL, 0);
            fcntl(client_sock, F_SETFL, flags | O_NONBLOCK);
            // Inner client loop: non-blocking recv polling
            uint8_t rx_buffer[RX_BUFFER_SIZE];
            const TickType_t client_loop_delay = pdMS_TO_TICKS(10); // 10ms quick check
            int drop_count = 0; // Track dropped bytes for potential connection close
            while (active_client.active && client_sock >= 0) {
                int total_read = 0;
                int len = 0; // Declare len outside do-while for scope
                // Drain loop: continuous recv until EAGAIN
                do {
                    len = recv(client_sock, rx_buffer, sizeof(rx_buffer), MSG_DONTWAIT);
                    if (len > 0) {
                        ESP_LOGI(TAG, "Recv %d bytes from " IPSTR ":%d", len, IP2STR((ip4_addr_t *)&active_client.addr.sin_addr), ntohs(active_client.addr.sin_port));
                        // ESP_LOG_BUFFER_HEX(TAG, rx_buffer, len);
                        // Append to RX ring
                        esp_err_t append_ret = rx_ring_append(rx_buffer, len);
                        if (append_ret != ESP_OK) {
                            ESP_LOGW(TAG, "RX append failed, dropping %d bytes", len);
                            drop_count += len;
                            if (drop_count > 1024 * 1024) { // If >1MB dropped, close to avoid waste
                                ESP_LOGE(TAG, "Excessive drops (%d bytes), closing connection", drop_count);
                                break;
                            }
                        } else {
                            ESP_LOGI(TAG, "Appended %d bytes to RX ring (now len=%zu)", len, rx_ring.len);
                            drop_count = 0; // Reset on success
                        }
                        total_read += len;
                        esp_task_wdt_reset(); // Reset post-recv
                    } else if (len == 0) {
                        ESP_LOGW(TAG, "Recv EOF from client");
                        break; // Connection closed
                    } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        ESP_LOGE(TAG, "Recv error: %d", errno);
                        break;
                    }
                } while (len > 0 && total_read < (64 * 1024)); // Limit total read per loop to ~64KB

                if (drop_count > 0) {
                    ESP_LOGW(TAG, "Dropped %d bytes this cycle", drop_count);
                }

                // Idle: heartbeat check (optional)
                if (total_read == 0) {
                    // Optional: check last_heartbeat, send ping if needed
                    esp_task_wdt_reset(); // Every 10ms
                }

                vTaskDelay(client_loop_delay); // Yield every 10ms
            }
            ESP_LOGI(TAG, "Client disconnected (sock=%d), drops=%d", client_sock, drop_count);
            xSemaphoreTake(client_mutex, portMAX_DELAY);
            active_client.sock = -1;
            active_client.active = false;
            xSemaphoreGive(client_mutex);
            shutdown(client_sock, SHUT_RDWR); // Graceful close
            close(client_sock);
            esp_task_wdt_reset(); // Post-disconnect
        } else { // Accept timeout
            esp_task_wdt_reset(); // Outer loop reset
        }
        vTaskDelay(loop_delay);
    }
    close(sock);
    vTaskDelete(NULL);
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

    // Modified: No header, no CRC, no escaping; directly copy original payload to frame
    memcpy(frame, payload, payload_len);  // Direct copy

    // Skip all other logic: no header build, no seq, no CRC computation, no escaping
    size_t frame_len = payload_len;  // Original length

    // 日志：确认构建
    ESP_LOGI(TAG, "Built control frame: type=0x%02X, seq=%u, payload_len=%u, crc=0x%04X, total_frame=%zu bytes", 
             type, 0, payload_len, 0, frame_len);

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

    xTaskCreatePinnedToCore(tcp_server_task, "tcp_server", 16384, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(usb_to_tcp_task, "usb_to_tcp", 12288, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(send_task, "send_task", 12288, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(parse_and_usb_task, "parse_usb", 12288, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(button_task, "button_task", 4096, NULL, 2, NULL, 0);  // Core 0 for GPIO
    xTaskCreatePinnedToCore(led_task, "led_task", 2048, NULL, 1, NULL, 0);
    ESP_LOGI(TAG, "Tasks created, app_main complete");
    
}
