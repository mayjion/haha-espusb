#include "tusb_composite_main.h"
#include <sys/select.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include "esp_task_wdt.h" // WDT Support
#include "esp_wifi_types.h" // WiFi Types
#include "lwip/inet.h"
static const char *TAG = "example_main";
static uint8_t cdc_rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
static QueueHandle_t app_queue;
// Removed unused tcp_to_usb_queue
static char current_ssid[32] = DEFAULT_ESP_WIFI_SSID;
static char current_password[64] = DEFAULT_ESP_WIFI_PASS;
static bool factory_state = true;
bool is_tcp_connected = false;
bool is_wifi_connected = false;
static int tcp_sock = -1;
static wifi_config_t wifi_config;
// Statistics counters: Used for monitoring packet loss
static size_t total_received_bytes = 0;
static size_t total_forwarded_bytes = 0;
static size_t lost_bytes = 0;
// Throughput counters (reset every interval)
static size_t rx_throughput_bytes = 0; // TCP -> USB
static size_t tx_throughput_bytes = 0; // USB -> TCP
static size_t total_tcp_rx_bytes = 0; // From TCP (forwarded to USB)
static size_t total_usb_rx_bytes = 0; // From USB (forwarded to TCP)
static size_t total_forwarded_to_usb = 0;
static size_t total_forwarded_to_tcp = 0;
static size_t lost_to_tcp = 0; // Drops USB->TCP
static size_t lost_to_usb = 0; // Drops TCP->USB
void print_stats(void) {
    float loss_tcp = total_usb_rx_bytes > 0 ? (100.0 * lost_to_tcp / total_usb_rx_bytes) : 0;
    float loss_usb = total_tcp_rx_bytes > 0 ? (100.0 * lost_to_usb / total_tcp_rx_bytes) : 0;
    ESP_LOGI(TAG, "STATS: TCP->USB Rx=%zu Fwd=%zu Lost=%.0f%% | USB->TCP Rx=%zu Fwd=%zu Lost=%.0f%%",
             total_tcp_rx_bytes, total_forwarded_to_usb, loss_usb,
             total_usb_rx_bytes, total_forwarded_to_tcp, loss_tcp);
    ESP_LOGI(TAG, "STATS: Received=%zu B, Forwarded=%zu B, Lost=%zu B (%.2f%%)",
             total_received_bytes, total_forwarded_bytes, lost_bytes,
             total_received_bytes > 0 ? (100.0 * lost_bytes / total_received_bytes) : 0);
}
// Device descriptor
static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0xCafe,
    .idProduct = 0x4001,
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
    (const char[]){0x09, 0x04},
    "Manufacturer",
    "ESP32-S3 CDC Device",
    "1234567890",
    "CDC Interface"
};
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event) {
    size_t rx_size = 0;
    esp_err_t ret = tinyusb_cdcacm_read(itf, cdc_rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK && rx_size > 0) {
        app_message_t msg = { .buf_len = rx_size, .itf = itf };
        memcpy(msg.buf, cdc_rx_buf, rx_size);
        // Block up to 20ms for queue space (backpressure to USB)
        if (xQueueSend(app_queue, &msg, pdMS_TO_TICKS(20)) != pdTRUE) {
            ESP_LOGW(TAG, "Queue full after 20ms wait, packet dropped (%zu bytes)", rx_size);
            lost_bytes += rx_size;
        } else {
            // Increment TX throughput (USB received, to be forwarded to TCP)
            tx_throughput_bytes += rx_size;
            total_usb_rx_bytes += rx_size;
        }
    }
}
void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event) {
    if (!event->line_state_changed_data.dtr || !event->line_state_changed_data.rts) {
        tud_cdc_n_write_clear(TINYUSB_CDC_ACM_0);
        tud_cdc_n_read_flush(TINYUSB_CDC_ACM_0);
    }
}
static void send_to_server(const uint8_t *data, size_t len) {
    if (!is_tcp_connected || tcp_sock < 0 || len == 0) return;
    size_t remaining = len;
    size_t total_sent = 0;
    bool send_success = true;
    int max_retries = 20;
    int retry_count = 0;
    int base_delay_ms = 1; // Start shorter
    TickType_t stall_start = xTaskGetTickCount(); // Track stall duration
    while (remaining > 0 && send_success && retry_count < max_retries) {
        // Quick writability check
        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(tcp_sock, &writefds);
        struct timeval tv = { .tv_sec = 0, .tv_usec = 1000 }; // 1ms poll
        if (select(tcp_sock + 1, NULL, &writefds, NULL, &tv) > 0 && FD_ISSET(tcp_sock, &writefds)) {
            // Writable: try send
            ssize_t sent = send(tcp_sock, data + total_sent, remaining, MSG_DONTWAIT | MSG_NOSIGNAL);
            if (sent < 0) {
                int err = errno;
                if (err == EAGAIN || err == EWOULDBLOCK) {
                    retry_count++;
                    int delay_ms = base_delay_ms << (retry_count / 5); // Exponential: 1,1,1,1,1,2,2,...
                    vTaskDelay(pdMS_TO_TICKS(delay_ms));
                    continue;
                } else {
                    ESP_LOGW(TAG, "TCP send error: %s (errno:%d), closing", strerror(err), err);
                    close(tcp_sock);
                    tcp_sock = -1;
                    is_tcp_connected = false;
                    send_success = false;
                    break;
                }
            } else if (sent == 0) {
                ESP_LOGI(TAG, "Server closed during send");
                close(tcp_sock);
                tcp_sock = -1;
                is_tcp_connected = false;
                send_success = false;
                break;
            } else {
                total_sent += sent;
                remaining -= sent;
                retry_count = 0; // Reset on progress
                stall_start = xTaskGetTickCount(); // Reset stall timer
            }
        } else {
            // Not writable: short backoff
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        // Safety: Close if stalled > STALL_TIMEOUT_MS
        if ((xTaskGetTickCount() - stall_start) > pdMS_TO_TICKS(STALL_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "TCP stalled >%dms, closing", STALL_TIMEOUT_MS);
            close(tcp_sock);
            tcp_sock = -1;
            is_tcp_connected = false;
            send_success = false;
            break;
        }
    }
    size_t sent = len - remaining;
    total_forwarded_to_tcp += sent;
    if (!send_success || total_sent < len) {
        lost_to_tcp += (len - total_sent);
        lost_bytes += (len - total_sent);
        ESP_LOGW(TAG, "Failed to send %zu/%zu to server (retries:%d)", total_sent, len, retry_count);
    } else {
        ESP_LOGD(TAG, "Sent %zu bytes to server", len);
    }
}
void tinyusb_cdc_line_coding_changed_callback(int itf, cdcacm_event_t *event) {
    cdcacm_event_line_coding_changed_data_t *coding = &event->line_coding_changed_data;
    char message[128];
    snprintf(message, sizeof(message), "BAUD=%lu,STOPBIT=%d,PARITY=%d,DATABIT=%d\r\n",
             coding->p_line_coding->bit_rate, coding->p_line_coding->stop_bits,
             coding->p_line_coding->parity, coding->p_line_coding->data_bits);
    send_to_server((uint8_t *)message, strlen(message));
}
static esp_err_t load_wifi_credentials(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed: %s, using default credentials", esp_err_to_name(ret));
        return ret;
    }
    size_t ssid_len = sizeof(current_ssid);
    bool ssid_found = (nvs_get_str(nvs_handle, NVS_KEY_SSID, current_ssid, &ssid_len) == ESP_OK);
    size_t pass_len = sizeof(current_password);
    bool pass_found = (nvs_get_str(nvs_handle, NVS_KEY_PASS, current_password, &pass_len) == ESP_OK);
    nvs_close(nvs_handle);
    factory_state = !(ssid_found && pass_found); // Factory if either missing
    if (!ssid_found) {
        strlcpy(current_ssid, DEFAULT_ESP_WIFI_SSID, sizeof(current_ssid));
    }
    if (!pass_found) {
        strlcpy(current_password, DEFAULT_ESP_WIFI_PASS, sizeof(current_password));
    }
    // Security: Never log actual password
    ESP_LOGI(TAG, "Loaded WiFi credentials: SSID=%s, Factory=%s", current_ssid, factory_state ? "yes" : "no");
    return ESP_OK;
}
static esp_err_t save_wifi_credentials(const char *ssid, const char *password) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = nvs_set_str(nvs_handle, NVS_KEY_SSID, ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS set SSID failed: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    ret = nvs_set_str(nvs_handle, NVS_KEY_PASS, password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS set password failed: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Saved WiFi credentials: SSID=%s", ssid); // No password log
    return ESP_OK;
}
static esp_err_t clear_nvs_config(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = nvs_erase_key(nvs_handle, NVS_KEY_SSID);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "NVS erase SSID failed: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    ret = nvs_erase_key(nvs_handle, NVS_KEY_PASS);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "NVS erase password failed: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "NVS config cleared, reverting to factory settings");
    return ESP_OK;
}
static void update_wifi_config(const char *ssid, const char *password) {
    memset(&wifi_config, 0, sizeof(wifi_config));
    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    wifi_config.sta.threshold.rssi = -127;
    wifi_config.sta.channel = ESP_WIFI_CHANNEL; // Use define from header
}
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        is_wifi_connected = false;
        ESP_LOGI(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        is_wifi_connected = true;
    }
}
static void wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    load_wifi_credentials();
    update_wifi_config(current_ssid, current_password);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    esp_wifi_set_max_tx_power(78);
    ESP_LOGI(TAG, "WiFi STA init with SSID: %s, connecting... (Ch=%d, MaxTX=20dBm)", current_ssid, ESP_WIFI_CHANNEL);
}
static void button_task(void *pvParameters) {
    TickType_t last_press_time = 0;
    bool last_state = true;
    while (1) {
        bool current_state = gpio_get_level(BUTTON_GPIO);
        if (last_state && !current_state) {
            last_press_time = xTaskGetTickCount();
        } else if (!last_state && current_state) { // Fixed: was !current_state, should detect release for duration
            TickType_t press_duration = xTaskGetTickCount() - last_press_time;
            ESP_LOGI(TAG, "BUTTON PRESSED: %d ms", press_duration / portTICK_PERIOD_MS);
            if (press_duration >= pdMS_TO_TICKS(BUTTON_LONG_PRESS_MS)) {
                esp_err_t ret = clear_nvs_config();
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Factory reset successful, restarting");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                } else {
                    ESP_LOGE(TAG, "Factory reset failed: %s", esp_err_to_name(ret));
                }
            }
        }
        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
    }
}
static void usb_to_tcp_task(void *pvParameters) {
    app_message_t msg;
    while (1) {
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY) == pdTRUE) {
            send_to_server(msg.buf, msg.buf_len);
        }
    }
}
static esp_err_t forward_to_usb(const uint8_t *data, size_t len) {
    size_t offset = 0;
    size_t forwarded_in_call = 0;
    esp_err_t overall_ret = ESP_OK;
    while (offset < len) {
        size_t to_write = len - offset;
        // Limit single write size to CDC_TX_CHUNK_SIZE
        if (to_write > CDC_TX_CHUNK_SIZE) to_write = CDC_TX_CHUNK_SIZE;
        size_t queued = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, data + offset, to_write);
        if (queued < to_write) {
            ESP_LOGW(TAG, "USB queue only accepted %zu/%zu bytes for chunk", queued, to_write);
            to_write = queued; // Only advance queued portion
        }
        esp_err_t flush_ret = ESP_FAIL;
        // For small packets (<64 bytes) use shorter timeout and fewer retries for low latency
        int max_retries = (to_write < 64) ? 3 : 10;
        TickType_t flush_timeout = (to_write < 64) ? pdMS_TO_TICKS(5) : pdMS_TO_TICKS(20);
        for (int retry = 0; retry < max_retries; retry++) {
            flush_ret = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, flush_timeout);
            if (flush_ret == ESP_OK) break;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (flush_ret != ESP_OK) {
            ESP_LOGE(TAG, "USB flush failed for %zu bytes after %d retries", to_write, max_retries);
            overall_ret = ESP_FAIL;
            lost_to_usb += to_write; // Count TCP->USB drops
            lost_bytes += to_write;
        } else {
            forwarded_in_call += to_write;
        }
        offset += to_write;
    }
    total_forwarded_to_usb += forwarded_in_call;
    total_forwarded_bytes += forwarded_in_call;
    if (overall_ret != ESP_OK) {
        ESP_LOGE(TAG, "Partial failure forwarding %zu/%zu bytes to USB", forwarded_in_call, len);
    } else {
        ESP_LOGD(TAG, "Successfully forwarded %zu bytes to USB", len);
        rx_throughput_bytes += len;
    }
    return overall_ret;
}
static void tcp_client_task(void *pvParameters) {
    esp_task_wdt_add(NULL);
    while (1) {
        esp_task_wdt_reset();
        if (!is_wifi_connected) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        tcp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (tcp_sock < 0) {
            ESP_LOGE(TAG, "Socket create failed: %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        int nodelay = 1;
        setsockopt(tcp_sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
        int bufsize = 32768;
        setsockopt(tcp_sock, SOL_SOCKET, SO_SNDBUF, &bufsize, sizeof(bufsize));
        setsockopt(tcp_sock, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));
        int flags = fcntl(tcp_sock, F_GETFL, 0);
        fcntl(tcp_sock, F_SETFL, flags | O_NONBLOCK);
        struct sockaddr_in dest_addr = { .sin_addr.s_addr = inet_addr(SERVER_IP), .sin_family = AF_INET, .sin_port = htons(PORT) };
        int err = connect(tcp_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0 && errno != EINPROGRESS) {
            close(tcp_sock); tcp_sock = -1;
            esp_task_wdt_reset(); // Reset before delay
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        esp_task_wdt_reset(); // Reset after connect start
        fd_set writefds; FD_ZERO(&writefds); FD_SET(tcp_sock, &writefds);
        struct timeval timeout = { .tv_sec = 3, .tv_usec = 0 }; // Reduced from 5s to 3s to fit within WDT
        err = select(tcp_sock + 1, NULL, &writefds, NULL, &timeout);
        esp_task_wdt_reset(); // Reset after select
        if (err <= 0 || !FD_ISSET(tcp_sock, &writefds)) {
            close(tcp_sock); tcp_sock = -1;
            vTaskDelay(pdMS_TO_TICKS(2000)); // Increased reconnect delay to 2s
            continue;
        }
        // Add larger SNDBUF
        int snd_bufsize = 256 * 1024; // 256KB send buffer
        setsockopt(tcp_sock, SOL_SOCKET, SO_SNDBUF, &snd_bufsize, sizeof(snd_bufsize));
        ESP_LOGI(TAG, "Set SO_SNDBUF to 256KB");
        // Further optimize RCVBUF for high ingress throughput
        int rcv_bufsize = 512 * 1024; // 512KB receive buffer
        setsockopt(tcp_sock, SOL_SOCKET, SO_RCVBUF, &rcv_bufsize, sizeof(rcv_bufsize));
        ESP_LOGI(TAG, "Set SO_RCVBUF to 512KB");
        is_tcp_connected = true;
        ESP_LOGI(TAG, "TCP connected to %s:%d", SERVER_IP, PORT);
        uint8_t rx_buf[RX_BUFFER_SIZE]; // Fixed: was rx_buffer
        TickType_t last_stat_time = xTaskGetTickCount();
        while (is_tcp_connected && tcp_sock >= 0) {
            esp_task_wdt_reset();
            if (tcp_sock < 0) {
                is_tcp_connected = false;
                break;
            }
            fd_set readfds; FD_ZERO(&readfds); FD_SET(tcp_sock, &readfds);
            // Increased poll to 100us to reduce CPU overhead while maintaining low latency
            struct timeval tv = { .tv_sec = 0, .tv_usec = 100 };
            int sel = select(tcp_sock + 1, &readfds, NULL, NULL, &tv);
            if (tcp_sock < 0) {
                is_tcp_connected = false;
                break;
            }
            if (sel > 0 && FD_ISSET(tcp_sock, &readfds)) {
                // Non-blocking recv loop to drain as much as possible in one iteration
                ssize_t total_received = 0;
                while (total_received < sizeof(rx_buf)) {
                    ssize_t len = recv(tcp_sock, rx_buf + total_received, sizeof(rx_buf) - total_received, MSG_DONTWAIT);
                    if (len > 0) {
                        total_received += len;
                    } else if (len == 0) {
                        // Server closed
                        ESP_LOGW(TAG, "Server closed connection");
                        is_tcp_connected = false;
                        break;
                    } else {
                        int recv_err = errno;
                        if (recv_err == EAGAIN || recv_err == EWOULDBLOCK) {
                            // No more data
                            break;
                        } else if (recv_err == EBADF) {
                            // Socket closed by other task
                            is_tcp_connected = false;
                            break;
                        } else {
                            // Error
                            ESP_LOGE(TAG, "Recv error: %d", recv_err);
                            is_tcp_connected = false;
                            break;
                        }
                    }
                }
                if (total_received > 0) {
                    total_tcp_rx_bytes += total_received;
                    total_received_bytes += total_received;
                    // Forward data to USB
                    esp_err_t ret = forward_to_usb(rx_buf, total_received); // Fixed: correct buffer, args, no 'i'
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to forward %zd bytes to USB", total_received);
                    }
                    if (xTaskGetTickCount() - last_stat_time > pdMS_TO_TICKS(10000)) {
                        print_stats();
                        last_stat_time = xTaskGetTickCount();
                    }
                }
            } else if (sel < 0) {
                int sel_err = errno;
                if (sel_err == EBADF) {
                    // Socket closed by other task
                    is_tcp_connected = false;
                } else {
                    ESP_LOGE(TAG, "Select error: %d", sel_err);
                    is_tcp_connected = false;
                }
                break;
            }
            // Minimal yield only if no data, to prioritize forwarding
            if (sel == 0) {
                vTaskDelay(pdMS_TO_TICKS(0));
            }
        }
        if (tcp_sock >= 0) {
            close(tcp_sock);
        }
        tcp_sock = -1;
        is_tcp_connected = false;
        ESP_LOGI(TAG, "TCP disconnected, retry in 2s");
        vTaskDelay(pdMS_TO_TICKS(2000)); // Increased to 2s
    }
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
    esp_task_wdt_add(NULL); // Subscribe to WDT
    while (1) {
        esp_task_wdt_reset();
        if (is_tcp_connected) {
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (is_wifi_connected) {
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}
static esp_err_t init_system(void) {
    // Deinit existing WDT to allow reconfiguration
    esp_err_t ret_deinit = esp_task_wdt_deinit();
    if (ret_deinit != ESP_OK && ret_deinit != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Task WDT deinit failed: %s", esp_err_to_name(ret_deinit));
    }
    // Reinitialize Task Watchdog with 30s timeout (increased for USB task safety)
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 30 * 1000,  // 30 seconds
        .idle_core_mask = 0,  // Do not monitor idle tasks (prevents false triggers)
        .trigger_panic = false  // No panic on timeout, just log
    };
    esp_err_t ret = esp_task_wdt_init(&twdt_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Task WDT init failed: %s, using default", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Task WDT initialized with 30s timeout, idle monitoring disabled");
    }
    // LED GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(LED_GPIO, 0); // Initial off, LED task will handle
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
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    button_init();
    size_t free_heap = esp_get_free_heap_size();
    ESP_LOGI(TAG, "Free heap before queue creation: %zu bytes", free_heap);
    app_queue = xQueueCreate(QUEUE_SIZE, sizeof(app_message_t));
    if (!app_queue) {
        ESP_LOGE(TAG, "Failed to create app_queue, free heap: %zu bytes", free_heap);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "app_queue created successfully");
    // Reset all counters
    total_received_bytes = 0;
    total_forwarded_bytes = 0;
    lost_bytes = 0;
    total_tcp_rx_bytes = 0;
    total_usb_rx_bytes = 0;
    total_forwarded_to_usb = 0;
    total_forwarded_to_tcp = 0;
    lost_to_tcp = 0;
    lost_to_usb = 0;
    rx_throughput_bytes = 0;
    tx_throughput_bytes = 0;
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
    // Do not subscribe usb_dev to WDT; tud_task() may block sporadically without easy reset points
    vTaskDelay(pdMS_TO_TICKS(100));
    while (1) {
        tud_task();
        vTaskDelay(1);
    }
}
void throughput_logger_task(void *pvParameters) {
    esp_task_wdt_add(NULL); // Subscribe to WDT
    TickType_t last_log = xTaskGetTickCount();
    while (1) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(THROUGHPUT_LOG_INTERVAL));
        TickType_t now = xTaskGetTickCount();
        float elapsed_sec = (now - last_log) * portTICK_PERIOD_MS / 1000.0f;
        float rx_kbps = (rx_throughput_bytes / 1024.0f) / elapsed_sec;
        float tx_kbps = (tx_throughput_bytes / 1024.0f) / elapsed_sec;
        ESP_LOGI(TAG, "Throughput: RX=%.1f KB/s, TX=%.1f KB/s (over %.2f s)", rx_kbps, tx_kbps, elapsed_sec);
        rx_throughput_bytes = 0;
        tx_throughput_bytes = 0; // Reset counters
        last_log = now;
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
    xTaskCreate(usb_device_task, "usb_dev", 8192, NULL, 10, NULL);
    xTaskCreate(tcp_client_task, "tcp_client", 8192, NULL, 8, NULL);
    xTaskCreate(usb_to_tcp_task, "usb_to_tcp", 8192, NULL, 9, NULL); // Higher prio
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
    xTaskCreate(led_task, "led_task", 1024, NULL, 1, NULL);
    xTaskCreatePinnedToCore(throughput_logger_task, "throughput_log", 4096, NULL, 3, NULL, 0);
    ESP_LOGI(TAG, "Tasks created, app_main complete");
}
