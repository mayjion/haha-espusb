#include "tcp_client.h"
#include "protocol.h"  // For frame building/parsing
#include "usbhandle.h"  // For tx_ring and app_queue
#include <string.h>    // For memset, memcpy
#include <sys/socket.h> // For send/recv

static const char *TAG = "tcpclient";
client_t active_client = {0};
SemaphoreHandle_t client_mutex = NULL;  // For thread-safe access to active_client
tx_ring_t tx_ring = {0};  // Shared with USB, initialized here or in usbhandle
QueueHandle_t app_queue;  // Shared with USB
rx_ring_t rx_ring = {0};  // RX ring
SemaphoreHandle_t rx_mutex = NULL;  // For rx_ring

void wdt_init(uint32_t timeout_ms) {  // uint8_t -> uint32_t
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
        .trigger_panic = true,
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

    // WiFi init (STA mode for client)
    wifi_config_init();  // Full init
    wifi_init_sta();     // Start STA connection

    // WDT init
    wdt_init(30000);

    // Mutex and shared resources
    client_mutex = xSemaphoreCreateMutex();
    if (client_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create client_mutex");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "client_mutex created OK");

    // Ring buffer init: Dynamic heap alloc
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

size_t rx_ring_consume(uint8_t *buf, size_t max_len) {
    if (buf == NULL || max_len == 0) return 0;  // Null guard vs MMU fault
    if (xSemaphoreTake(rx_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return 0;
    size_t to_read = (rx_ring.len < max_len) ? rx_ring.len : max_len;
    if (to_read == 0 || rx_ring.buf == NULL) {
        xSemaphoreGive(rx_mutex);
        return 0;
    }
    size_t read = 0;
    size_t pos = rx_ring.tail;
    while (read < to_read) {
        if (pos >= rx_ring.size) pos = 0;  // Bounds
        buf[read] = rx_ring.buf[pos];
        pos = (pos + 1) % rx_ring.size;
        read++;
    }
    rx_ring.tail = pos;
    rx_ring.len -= to_read;
    xSemaphoreGive(rx_mutex);
    return to_read;
}

esp_err_t send_control_to_tcp(uint8_t type, const uint8_t *payload, uint16_t payload_len) {
    xSemaphoreTake(client_mutex, portMAX_DELAY);
    if (!active_client.active) {
        ESP_LOGW(TAG, "TCP not connected, dropping control msg type=0x%02X", type);
        xSemaphoreGive(client_mutex);
        return ESP_FAIL;
    }
    esp_err_t ret = send_reliable(&active_client, type, payload, payload_len);
    xSemaphoreGive(client_mutex);
    return ret;
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
    const TickType_t send_interval = pdMS_TO_TICKS(SEND_CHECK_INTERVAL_MS);

    while (1) {
        xSemaphoreTake(client_mutex, portMAX_DELAY);
        if (!active_client.active || tx_ring.len == 0) {
            xSemaphoreGive(client_mutex);
            vTaskDelay(send_interval);
            esp_task_wdt_reset();
            continue;
        }

        // Consume from ring
        size_t to_send = (tx_ring.len > MAX_TX_SIZE) ? MAX_TX_SIZE : tx_ring.len;
        uint8_t send_buf[MAX_TX_SIZE];
        size_t pos = tx_ring.tail;
        for (size_t i = 0; i < to_send; ++i) {
            send_buf[i] = tx_ring.buf[pos];
            pos = (pos + 1) % tx_ring.size;
        }
        tx_ring.tail = pos;
        tx_ring.len -= to_send;
        xSemaphoreGive(client_mutex);

        // Send reliable DATA frame
        active_client.pending = true;
        active_client.pending_type = PROTO_TYPE_DATA;
        active_client.pending_seq = active_client.seq_tx++;
        memcpy(active_client.pending_payload, send_buf, to_send);
        active_client.pending_len = to_send;

        esp_err_t send_ret = send_reliable(&active_client, PROTO_TYPE_DATA, send_buf, to_send);
        if (send_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send DATA seq=%d len=%zu", active_client.pending_seq, to_send);
        } else {
            ESP_LOGD(TAG, "Sent DATA seq=%d len=%zu", active_client.pending_seq, to_send);
        }

        // Heartbeat every interval
        if (xTaskGetTickCount() - active_client.last_heartbeat > pdMS_TO_TICKS(PROTO_HEARTBEAT_INTERVAL)) {
            uint8_t hb_payload[1] = {0x00};  // Simple HB
            send_reliable(&active_client, PROTO_TYPE_HEARTBEAT, hb_payload, 1);
            active_client.last_heartbeat = xTaskGetTickCount();
        }

        vTaskDelay(send_interval);
        esp_task_wdt_reset();
    }
}

void parse_and_usb_task(void *pvParameters) {
    esp_task_wdt_add(NULL);
    uint8_t parse_buf[FRAME_BUFFER_SIZE];
    uint8_t payload[MAX_TX_SIZE];
    const size_t max_frames_per_loop = 5;  // Reduced for stack safety under bursts
    size_t frames_processed = 0;
    const TickType_t parse_delay = pdMS_TO_TICKS(1);

    while (1) {
        // Null check on ring (prevent MMU fault)
        if (rx_ring.buf == NULL || rx_ring.size == 0) {
            ESP_LOGE(TAG, "RX ring invalid (null buf), skipping");
            esp_task_wdt_reset();
            vTaskDelay(parse_delay);
            continue;
        }

        size_t avail = rx_ring_consume(parse_buf, sizeof(parse_buf));
        if (avail == 0) {
            esp_task_wdt_reset();
            vTaskDelay(parse_delay);
            continue;
        }

        ESP_LOGI(TAG, "Consumed %zu bytes from RX ring for parsing", avail);

        size_t parsed = 0;
        while (parsed < avail && frames_processed < max_frames_per_loop) {
            uint8_t type;
            uint16_t seq, pay_len, crc;
            frame_parse_status_t status = extract_next_frame(&parse_buf[parsed], avail - parsed, &type, &seq, &pay_len, payload, &crc);
            if (status > 0) {
                parsed += (size_t)status;
                frames_processed++;
                ESP_LOGI(TAG, "Parsed frame: type=0x%02X seq=%u len=%u crc=0x%04X", type, seq, pay_len, crc);

                if (type == PROTO_TYPE_DATA) {
                    // Null check payload
                    if (payload == NULL || pay_len == 0) {
                        ESP_LOGW(TAG, "Invalid payload for DATA frame seq=%u", seq);
                        continue;
                    }

                    size_t written = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, payload, pay_len);
                    esp_err_t flush_ret = ESP_FAIL;
                    for (int retry = 0; retry < 3; retry++) {
                        flush_ret = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(CDC_FLUSH_TIMEOUT_MS));
                        if (flush_ret == ESP_OK && written == pay_len) break;
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }

                    if (flush_ret != ESP_OK || written != pay_len) {
                        ESP_LOGE(TAG, "Failed to forward %d bytes from server to USB after retries", pay_len);
                    }
                    
                    ESP_LOGD(TAG, "Forwarded DATA frame to USB (no app ACK sent; using TCP)");
                } else if (type == PROTO_TYPE_CMD) {
                    ESP_LOGI(TAG, "CMD received from server, len=%u", pay_len);
                }
            } else if (status == FRAME_INCOMPLETE) {
                ESP_LOGD(TAG, "Incomplete frame at offset %zu, dropping %zu bytes", parsed, avail - parsed);
                parsed = avail;
                break;
            } else {
                ESP_LOGW(TAG, "Parse error at offset %zu, skipping est %u bytes", parsed, pay_len + 9);
                size_t skip = 7 + pay_len + 2;
                if (skip > avail - parsed) skip = avail - parsed;
                parsed += skip;
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            esp_task_wdt_reset();
        }
        esp_task_wdt_reset();
        frames_processed = 0;
        vTaskDelay(parse_delay);
    }
}
esp_err_t send_reliable(client_t *client, uint8_t type, const uint8_t *payload, uint16_t payload_len) {
    if (type != PROTO_TYPE_DATA) {
        // For non-DATA: seq=0, simple retry
        uint16_t seq = 0;
        uint8_t frame[FRAME_BUFFER_SIZE];
        size_t frame_len = build_escaped_frame(frame, type, seq, payload, payload_len, NULL);
        
        ESP_LOGD(TAG, "Built non-DATA frame len=%zu seq=0 type=0x%02x", frame_len, type);
        
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
        // For DATA: use pending seq, non-blocking send with timeout
        uint8_t frame[FRAME_BUFFER_SIZE];
        size_t frame_len = build_escaped_frame(frame, type, client->pending_seq, payload, payload_len, NULL);
        
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

        ESP_LOGD(TAG, "Sent DATA seq=%d len=%d (full %zu bytes)", client->pending_seq, payload_len, frame_len);
        return ESP_OK;
    }
}

void usb_to_tcp_task(void *pvParameters) {
    esp_task_wdt_add(NULL);
    app_message_t msg;
    const TickType_t queue_timeout = pdMS_TO_TICKS(25000);  // Reset every ~25s if idle (<30s WDT)

    while (1) {
        BaseType_t got = xQueueReceive(app_queue, &msg, queue_timeout);
        esp_task_wdt_reset();  // Always reset after wait/processing

        if (got == pdTRUE) {
            xSemaphoreTake(client_mutex, portMAX_DELAY);
            if (!active_client.active) {
                ESP_LOGW(TAG, "TCP not ready, dropping %zu USB bytes", msg.buf_len);
                xSemaphoreGive(client_mutex);
                continue;
            }

            // Check ring space
            size_t free_space = tx_ring.size - tx_ring.len;
            size_t to_write = (msg.buf_len > free_space) ? free_space : msg.buf_len;
            if (to_write == 0) {
                ESP_LOGW(TAG, "TX ring full (%zu/%zu), dropping %zu bytes", tx_ring.len, tx_ring.size, msg.buf_len);
                xSemaphoreGive(client_mutex);
                continue;
            }

            // Write to ring
            size_t pos = tx_ring.head;
            for (size_t i = 0; i < to_write; ++i) {
                tx_ring.buf[pos] = msg.buf[i];
                pos = (pos + 1) % tx_ring.size;
            }
            tx_ring.head = pos;
            tx_ring.len += to_write;
            ESP_LOGD(TAG, "Queued %zu USB bytes (ring: %zu/%zu)", to_write, tx_ring.len, tx_ring.size);

            xSemaphoreGive(client_mutex);

            if (tx_ring.len > (TX_BUFFER_SIZE * 0.8)) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        // No else: timeout auto-handled by reset above
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