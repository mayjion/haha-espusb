/* WiFi Station with USB CDC / UART0 to TCP Bridge (Optimized for ESP32-S2)
   Supports USB_CDC macro: 1=USB CDC mode (default), 0=UART0 mode.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "tusb_composite_main.h"
#include <sys/select.h>
#include <string.h>
#include "rom/ets_sys.h"

static uint32_t current_baud_rate = 115200;
static uint8_t current_stop_bits = UART_STOP_BITS_1;
static uint8_t current_parity = UART_PARITY_DISABLE;
static uint8_t current_data_bits = UART_DATA_8_BITS;
static bool is_tcp_connected = false;
static int tcp_sock = -1;
static uint8_t debug_mode = 0;

// FIX: Added missing declaration for WiFi retry counter
static int s_retry_num = 0;

// Reconnect backoff state
static uint32_t tcp_reconnect_delay_ms = TCP_RETRY_BASE_DELAY_MS;
static uint32_t last_reconnect_time = 0;

// FIX: Forward declaration for factory_reset (to resolve implicit declaration)
static void factory_reset(void);

// USB CDC globals
static uint8_t cdc_rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
static QueueHandle_t app_queue = NULL;  // For USB RX to TCP

// Device descriptor (Espressif official VID/PID for auto-detection)
static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x2886,  // Espressif VID
    .idProduct = 0x003e, // S2 USB Serial PID
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
    "Espressif",  // Manufacturer
    "ESP32-S2 CDC Client",  // Product
    "1234567890",
    "CDC Interface"
};

// Message struct for queue
typedef struct {
    uint8_t buf[UART_BUF_SIZE];
    size_t buf_len;
    int itf;
} app_message_t;

/* CRC32 computation for ESP32-S2 */
static uint32_t compute_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    const uint32_t poly = 0xEDB88320;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ poly;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFF;
}

/* Author code verification */
static void authorcodeverify(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for auth: %s", esp_err_to_name(err));
        while (1) {
            gpio_set_level(LED_GPIO, 1); // on (assuming active-high, adjust if needed)
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_GPIO, 0); // off
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }

    uint8_t mac[6];
    err = esp_wifi_get_mac(WIFI_IF_STA, mac);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get MAC: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        while (1) {
            gpio_set_level(LED_GPIO, 1); // on
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_GPIO, 0); // off
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }

    uint32_t computed_code = compute_crc32(mac, 6);

    uint32_t stored_code = 0;
    err = nvs_get_u32(nvs_handle, "auth_code", &stored_code);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // First time, write the code
        err = nvs_set_u32(nvs_handle, "auth_code", computed_code);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write auth_code: %s", esp_err_to_name(err));
            nvs_close(nvs_handle);
            while (1) {
                gpio_set_level(LED_GPIO, 1); // on
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(LED_GPIO, 0); // off
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
        }
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit auth_code: %s", esp_err_to_name(err));
        }
        ESP_LOGI(TAG, "Auth code written successfully");
        nvs_close(nvs_handle);
        return;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read auth_code: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        while (1) {
            gpio_set_level(LED_GPIO, 1); // on
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_GPIO, 0); // off
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }

    // Verify
    if (computed_code != stored_code) {
        ESP_LOGE(TAG, "Auth verification failed. Computed: 0x%08x, Stored: 0x%08x", computed_code, stored_code);
        nvs_close(nvs_handle);
        while (1) {
            gpio_set_level(LED_GPIO, 1); // on
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_GPIO, 0); // off
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }

    ESP_LOGI(TAG, "Auth verification passed");
    nvs_close(nvs_handle);
}

/* Initialize UART0 */
static void uart_init(uint32_t baud_rate, uint8_t stop_bits, uint8_t parity, uint8_t data_bits)
{
    if (USB_CDC) return;  // Skip UART init in USB mode
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = data_bits,
        .parity = parity,
        .stop_bits = stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_param_config(UART_NUM_0, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
    }
    // 增大 TX buffer 到 2048，避免 queue 满阻塞
    err = uart_driver_install(UART_NUM_0, UART_BUF_SIZE * 2, UART_BUF_SIZE * 4, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
    }
}

/* Check WiFi connection status */
static bool is_wifi_connected(void)
{
    return (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) != 0;
}

/* Initialize LED on GPIO17 */
static void led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LED GPIO config failed: %s", esp_err_to_name(err));
    }
    gpio_set_level(LED_GPIO, 0); // LED off by default
}

/* LED control task */
/* LED control task */
static void led_task(void *pvParameters)
{
    esp_task_wdt_add(NULL);  // Register this task with WDT
    while (1) {
        bool wifi_ok = is_wifi_connected();
        bool tcp_ok = is_tcp_connected;

        if (!wifi_ok) {
            // 慢闪: 亮500ms, 灭500ms
            gpio_set_level(LED_GPIO, 1); // on
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(LED_GPIO, 0); // off
            vTaskDelay(pdMS_TO_TICKS(500));
        } else if (!tcp_ok) {
            // 快闪: 亮200ms, 灭200ms
            gpio_set_level(LED_GPIO, 1); // on
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(LED_GPIO, 0); // off
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            // 常亮: on, 检查状态每1000ms
            gpio_set_level(LED_GPIO, 1); // on
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        esp_task_wdt_reset();
    }

    vTaskDelete(NULL);
}


/* Button task for long press factory reset */
static void button_task(void *pvParameters)
{
    esp_task_wdt_add(NULL);  // Register this task with WDT
    TickType_t last_press_time = 0;
    bool last_state = true;  // Assuming pull-up, idle high
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Button GPIO config failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
    }

    while (1) {
        bool current_state = gpio_get_level(BUTTON_GPIO);
        if (last_state && !current_state) {
            // Button pressed
            last_press_time = xTaskGetTickCount();
        } else if (!last_state && current_state) {
            // Button released
            TickType_t press_duration = xTaskGetTickCount() - last_press_time;
            if (press_duration >= pdMS_TO_TICKS(BUTTON_LONG_PRESS_MS)) {
                ESP_LOGI(TAG, "Long press detected: %d ms, initiating factory reset", press_duration / portTICK_PERIOD_MS);
                factory_reset();
            }
        }
        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
        esp_task_wdt_reset();  // Feed WDT here (loop end)
    }

    vTaskDelete(NULL);
}

/* Factory reset function */
static void factory_reset(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for factory reset: %s", esp_err_to_name(err));
        return;
    }

    // Preserve auth_code before erasing
    uint32_t auth_code = 0;
    esp_err_t auth_err = nvs_get_u32(nvs_handle, "auth_code", &auth_code);
    bool has_auth_code = (auth_err == ESP_OK);

    // Erase all except auth_code
    err = nvs_erase_all(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return;
    }
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS erase: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return;
    }

    // Restore auth_code if it existed
    if (has_auth_code) {
        err = nvs_set_u32(nvs_handle, "auth_code", auth_code);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to restore auth_code: %s", esp_err_to_name(err));
        } else {
            err = nvs_commit(nvs_handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to commit restored auth_code: %s", esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG, "Auth code preserved during factory reset");
            }
        }
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Factory reset completed, restarting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

/* Load configurations from NVS */
static esp_err_t load_nvs_config(uint32_t *baud_rate, uint8_t *stop_bits, uint8_t *parity, uint8_t *data_bits, char *ssid, size_t ssid_len, char *password, size_t pass_len, uint8_t *debug)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        *baud_rate = 115200;
        *stop_bits = UART_STOP_BITS_1;
        *parity = UART_PARITY_DISABLE;
        *data_bits = UART_DATA_8_BITS;
        strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, ssid_len);
        strlcpy(password, DEFAULT_ESP_WIFI_PASS, pass_len);
        *debug = 0;
        return err;
    }

    size_t len = ssid_len;
    err = nvs_get_str(nvs_handle, "wifi_ssid", ssid, &len);
    if (err != ESP_OK || len == 0) {
        strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, ssid_len);
    }
    len = pass_len;
    err = nvs_get_str(nvs_handle, "wifi_pass", password, &len);
    if (err != ESP_OK || len == 0) {
        strlcpy(password, DEFAULT_ESP_WIFI_PASS, pass_len);
    }
    err = nvs_get_u32(nvs_handle, "baud_rate", baud_rate);
    if (err != ESP_OK) {
        *baud_rate = 115200;
    }
    err = nvs_get_u8(nvs_handle, "stop_bits", stop_bits);
    if (err != ESP_OK) {
        *stop_bits = UART_STOP_BITS_1;
    }
    err = nvs_get_u8(nvs_handle, "parity", parity);
    if (err != ESP_OK) {
        *parity = UART_PARITY_DISABLE;
    }
    err = nvs_get_u8(nvs_handle, "data_bits", data_bits);
    if (err != ESP_OK) {
        *data_bits = UART_DATA_8_BITS;
    }
    err = nvs_get_u8(nvs_handle, "debug_mode", debug);
    if (err != ESP_OK) {
        *debug = 0;
    }
    nvs_close(nvs_handle);

    if (ssid[0] == '\0' || strlen(ssid) > 32 || !isprint((unsigned char)ssid[0])) {
        strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, ssid_len);
    }
    size_t pass_length = strlen(password);
    if (pass_length > 0 && (pass_length < 8 || pass_length > 64 || !isprint((unsigned char)password[0]))) {
        strlcpy(password, DEFAULT_ESP_WIFI_PASS, pass_len);
    }

    esp_log_level_set(TAG, *debug ? ESP_LOG_INFO : ESP_LOG_NONE);
    return ESP_OK;
}

/* Save configurations to NVS */
static void save_nvs_config(uint32_t baud_rate, uint8_t stop_bits, uint8_t parity, uint8_t data_bits, const char *ssid, const char *password, uint8_t debug)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for saving config: %s", esp_err_to_name(err));
        return;
    }
    if (ssid) {
        nvs_set_str(nvs_handle, "wifi_ssid", ssid);
    }
    if (password) {
        nvs_set_str(nvs_handle, "wifi_pass", password);
    }
    nvs_set_u32(nvs_handle, "baud_rate", baud_rate);
    nvs_set_u8(nvs_handle, "stop_bits", stop_bits);
    nvs_set_u8(nvs_handle, "parity", parity);
    nvs_set_u8(nvs_handle, "data_bits", data_bits);
    nvs_set_u8(nvs_handle, "debug_mode", debug);
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS config: %s", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
}

/* Print raw data (binary-safe) to UART0 or USB CDC */
static SemaphoreHandle_t raw_log_mutex = NULL;

static void raw_log_init(void)
{
    if (!raw_log_mutex) {
        raw_log_mutex = xSemaphoreCreateMutex();
        if (!raw_log_mutex) {
            ESP_LOGE(TAG, "Failed to create raw log mutex");
        }
    }
}

static void print_raw_data(const uint8_t *data, size_t len)
{
    if (!data || len == 0) return;

    // 喂 TWDT 前
    esp_task_wdt_reset();

    static bool initialized = false;
    if (!initialized) {
        raw_log_init();
        initialized = true;
    }

    if (USB_CDC) {
        // USB CDC output
        size_t queued = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, data, len);
        if (queued != len) {
            ESP_LOGW(TAG, "CDC write queue incomplete: %zu/%zu bytes", queued, len);
        }
        esp_err_t flush_ret = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(10));  // 10ms timeout
        if (flush_ret != ESP_OK) {
            ESP_LOGW(TAG, "CDC write flush failed: %s", esp_err_to_name(flush_ret));
        }
    } else {
        // Original UART output
        if (raw_log_mutex && xSemaphoreTake(raw_log_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            int written = uart_write_bytes(UART_NUM_0, (const char *)data, len);
            if (written != len) {
                ESP_LOGW(TAG, "UART write incomplete: %d/%zu bytes", written, len);
            }
            // 移除 uart_wait_tx_done()：避免阻塞，允许异步 TX
            xSemaphoreGive(raw_log_mutex);
        } else {
            uart_write_bytes(UART_NUM_0, (const char *)data, len);
            // 移除 uart_wait_tx_done()
        }
    }

    // 喂 TWDT 后
    esp_task_wdt_reset();
}

/* Non-blocking send with retry */
static esp_err_t nonblocking_send(int sock, const char *payload, size_t payload_len, int max_retries) {
    size_t sent = 0;
    while (sent < payload_len) {
        int bytes = send(sock, payload + sent, payload_len - sent, 0);
        if (bytes > 0) {
            sent += bytes;
            esp_task_wdt_reset();  // Feed during retries
            continue;
        }
        if (bytes < 0) {
            int lwip_errno = errno;
            if (lwip_errno == EAGAIN || lwip_errno == EWOULDBLOCK) {
                if (--max_retries <= 0) {
                    ESP_LOGW(TAG, "Send EAGAIN timeout after %d retries (buffer full?)", max_retries + 1);
                    return ESP_ERR_TIMEOUT;  // Transient
                }
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            } else {
                ESP_LOGE(TAG, "Send real error: errno %d", lwip_errno);
                return ESP_FAIL;  // Fatal
            }
        }
    }
    return ESP_OK;
}

/* TCP send data with optional response expectation */
static esp_err_t tcp_send_data(int sock, const char *payload, size_t payload_len, char *rx_buffer, size_t rx_buffer_size, bool expect_response)
{
    esp_err_t send_err = nonblocking_send(sock, payload, payload_len, 50);  // ~20ms retries
    if (send_err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Transient send timeout, returning non-fatal");
        return ESP_ERR_TIMEOUT;
    } else if (send_err != ESP_OK) {
        ESP_LOGE(TAG, "Fatal error during sending: %s", esp_err_to_name(send_err));
        return ESP_FAIL;
    }

    if (!expect_response) {
        return ESP_OK;
    }

    struct timeval timeout;
    timeout.tv_sec = RESPONSE_TIMEOUT_MS / 1000;
    timeout.tv_usec = (RESPONSE_TIMEOUT_MS % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    int len = recv(sock, rx_buffer, rx_buffer_size - 1, 0);
    if (len < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            ESP_LOGE(TAG, "No response from server within %d ms", RESPONSE_TIMEOUT_MS);
            return ESP_FAIL;
        }
        ESP_LOGE(TAG, "recv failed: errno %d", errno);
        return ESP_FAIL;
    }

    print_raw_data((uint8_t *)rx_buffer, len);
    return ESP_OK;
}

static esp_err_t parse_tcp_command(int sock, const uint8_t *rx_buffer, size_t len)
{
    if (!rx_buffer || len == 0) {
        return ESP_FAIL;
    }

    // 快速路径：如果含 \r\n，假设命令；否则 passthrough
    bool is_likely_command = (memchr(rx_buffer, '\r', len) != NULL || memchr(rx_buffer, '\n', len) != NULL);
    if (!is_likely_command) {
        print_raw_data(rx_buffer, len);
        return ESP_OK;
    }

    // Check if potential BAUD command
    if (len >= 5 && memcmp(rx_buffer, "BAUD=", 5) == 0) {
        // Check for embedded 0x00; if present, treat as transparent binary data
        if (memchr(rx_buffer, 0, len) != NULL) {
            print_raw_data(rx_buffer, len);
            return ESP_OK;
        }

        // Safe to make null-terminated copy
        char *buf = (char *)malloc(len + 1);
        if (!buf) {
            ESP_LOGE(TAG, "Failed to allocate buf for command");
            return ESP_FAIL;
        }
        memcpy(buf, rx_buffer, len);
        buf[len] = 0;

        // Trim to first \r\n if present
        char *cmd_end = strstr(buf, "\r\n");
        if (cmd_end) {
            *cmd_end = 0;
        }

        // Regex pattern
        const char *pattern = "^BAUD=([0-9]+),STOPBIT=([0-9]+),PARITY=([0-9]+),DATABIT=([0-9]+)(\r\n)?$";
        regex_t regex;
        regmatch_t matches[5];
        int ret = regcomp(&regex, pattern, REG_EXTENDED);
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to compile regex: %d", ret);
            const char *response = "Internal error: regex compilation failed\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }

        ret = regexec(&regex, buf, 5, matches, 0);
        if (ret != 0) {
            ESP_LOGE(TAG, "Invalid BAUD command format: no match for '%s'", buf);
            const char *response = "Invalid BAUD command format\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }

        // Extract and convert each field
        char *endptr;
        char baud_str[32] = {0};
        size_t baud_len = matches[1].rm_eo - matches[1].rm_so;
        if (baud_len >= sizeof(baud_str)) {
            ESP_LOGE(TAG, "BAUD value too long");
            const char *response = "Invalid baud rate\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        strncpy(baud_str, buf + matches[1].rm_so, baud_len);
        uint32_t new_baud = strtoul(baud_str, &endptr, 10);
        if (*endptr != 0) {
            ESP_LOGE(TAG, "Invalid BAUD value: non-numeric '%s'", baud_str);
            const char *response = "Invalid baud rate\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }

        // STOPBIT
        char stopbit_str[32] = {0};
        size_t stopbit_len = matches[2].rm_eo - matches[2].rm_so;
        if (stopbit_len >= sizeof(stopbit_str)) {
            ESP_LOGE(TAG, "STOPBIT value too long");
            const char *response = "Invalid stop bits\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        strncpy(stopbit_str, buf + matches[2].rm_so, stopbit_len);
        uint8_t raw_stop_bits = strtoul(stopbit_str, &endptr, 10);
        if (*endptr != 0) {
            ESP_LOGE(TAG, "Invalid STOPBIT value: non-numeric '%s'", stopbit_str);
            const char *response = "Invalid stop bits\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        uint8_t new_stop_bits = raw_stop_bits + 1;

        // PARITY
        char parity_str[32] = {0};
        size_t parity_len = matches[3].rm_eo - matches[3].rm_so;
        if (parity_len >= sizeof(parity_str)) {
            ESP_LOGE(TAG, "PARITY value too long");
            const char *response = "Invalid parity\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        strncpy(parity_str, buf + matches[3].rm_so, parity_len);
        uint8_t new_parity = strtoul(parity_str, &endptr, 10);
        if (*endptr != 0) {
            ESP_LOGE(TAG, "Invalid PARITY value: non-numeric '%s'", parity_str);
            const char *response = "Invalid parity\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        if (new_parity == 1) new_parity = 3;

        // DATABIT
        char databit_str[32] = {0};
        size_t databit_len = matches[4].rm_eo - matches[4].rm_so;
        if (databit_len >= sizeof(databit_str)) {
            ESP_LOGE(TAG, "DATABIT value too long");
            const char *response = "Invalid data bits\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        strncpy(databit_str, buf + matches[4].rm_so, databit_len);
        uint8_t raw_data_bits = strtoul(databit_str, &endptr, 10);
        if (*endptr != 0) {
            ESP_LOGE(TAG, "Invalid DATABIT value: non-numeric '%s'", databit_str);
            const char *response = "Invalid data bits\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }

        regfree(&regex);
        free(buf);

        // Validate parameters
        if (new_baud < 110 || new_baud > 2000000) {
            ESP_LOGE(TAG, "Invalid baud rate: %u", new_baud);
            const char *response = "Invalid baud rate\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            return ESP_FAIL;
        }

        if (new_stop_bits != UART_STOP_BITS_1 && new_stop_bits != UART_STOP_BITS_2) {
            ESP_LOGE(TAG, "Invalid stop bits: %u (raw value: %u)", new_stop_bits, raw_stop_bits);
            const char *response = "Invalid stop bits\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            return ESP_FAIL;
        }

        if (new_parity != UART_PARITY_DISABLE && new_parity != UART_PARITY_ODD && new_parity != UART_PARITY_EVEN) {
            ESP_LOGE(TAG, "Invalid parity: %u", new_parity);
            const char *response = "Invalid parity\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            return ESP_FAIL;
        }

        // raw_data_bits is from strtoul (unsigned), so check range before subtracting
        if (raw_data_bits < 5 || raw_data_bits > 8) {
            ESP_LOGE(TAG, "Invalid data bits: %u", raw_data_bits);
            const char *response = "Invalid data bits\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            return ESP_FAIL;
        }
        uint8_t new_data_bits = raw_data_bits - 5;

        // Apply UART configuration only if !USB_CDC
        if (!USB_CDC) {
            // Apply UART configuration
            uart_set_baudrate(UART_NUM_0, new_baud);
            uart_set_stop_bits(UART_NUM_0, new_stop_bits);
            uart_set_parity(UART_NUM_0, new_parity);
            uart_set_word_length(UART_NUM_0, new_data_bits);

            // Update global variables and save to NVS
            current_baud_rate = new_baud;
            current_stop_bits = new_stop_bits;
            current_parity = new_parity;
            current_data_bits = new_data_bits;
            save_nvs_config(new_baud, new_stop_bits, new_parity, new_data_bits, NULL, NULL, debug_mode);
        } else {
            ESP_LOGI(TAG, "BAUD command ignored in USB CDC mode (virtual serial)");
        }
        const char *response = "OK\r\n";
        nonblocking_send(sock, response, strlen(response), 3);
        return ESP_OK;
    } else if (len >= 5 && memcmp(rx_buffer, "WIFI=", 5) == 0) {
        // Similar fix for WIFI command: check for 0x00
        if (memchr(rx_buffer, 0, len) != NULL) {
            print_raw_data(rx_buffer, len);
            return ESP_OK;
        }

        // Safe to make null-terminated copy
        char *buf = (char *)malloc(len + 1);
        if (!buf) {
            ESP_LOGE(TAG, "Failed to allocate buf for command");
            return ESP_FAIL;
        }
        memcpy(buf, rx_buffer, len);
        buf[len] = 0;

        char *ssid = buf + 5;
        char *password = strchr(ssid, ',');
        if (password) {
            *password = 0;
            password++;
            char *end = strchr(password, '\r');
            if (end) *end = 0;

            if (strlen(ssid) == 0 || strlen(ssid) > 32 || !isprint((unsigned char)ssid[0])) {
                ESP_LOGE(TAG, "Invalid SSID from server: %s", ssid);
                const char *response = "Invalid SSID\r\n";
                nonblocking_send(sock, response, strlen(response), 3);
                free(buf);
                return ESP_FAIL;
            } else if (strlen(password) > 0 && (strlen(password) < 8 || strlen(password) > 64 || !isprint((unsigned char)password[0]))) {
                ESP_LOGE(TAG, "Invalid password from server");
                const char *response = "Invalid password\r\n";
                nonblocking_send(sock, response, strlen(response), 3);
                free(buf);
                return ESP_FAIL;
            } else {
                save_nvs_config(current_baud_rate, current_stop_bits, current_parity, current_data_bits, ssid, password, debug_mode);
                ESP_LOGI(TAG, "WiFi configuration from server successful: SSID=%s", ssid);
                free(buf);
                esp_restart();
                return ESP_OK;
            }
        } else {
            ESP_LOGE(TAG, "Invalid WIFI command format from server");
            const char *response = "Invalid WIFI command format\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            free(buf);
            return ESP_FAIL;
        }
    } else {
        // Not a command: transparent binary passthrough, send full len to UART
        print_raw_data(rx_buffer, len);
        return ESP_OK;
    }
}

static void tcp_receive_task(void *pvParameters)
{
    esp_task_wdt_add(NULL);  // Register this task with WDT
    uint8_t *rx_buffer = (uint8_t *) malloc(UART_BUF_SIZE);  // FIX: Changed to uint8_t* for binary
    if (!rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for receive buffer");
        vTaskDelete(NULL);
    }

    while (1) {
        if (!is_tcp_connected || tcp_sock == -1) {
            vTaskDelay(pdMS_TO_TICKS(100));  // Reduced from 500ms, but not too aggressive
            esp_task_wdt_reset();  // 喂狗
            continue;
        }

        int len = recv(tcp_sock, rx_buffer, UART_BUF_SIZE - 1, 0);
        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                vTaskDelay(pdMS_TO_TICKS(1));  // 1ms yield for non-blocking
                esp_task_wdt_reset();
                continue;
            }
            // Real disconnect: ECONNRESET, ETIMEDOUT, etc.
            ESP_LOGW(TAG, "TCP disconnect detected: errno %d, marking for reconnect...", errno);
            is_tcp_connected = false;
            close(tcp_sock);
            tcp_sock = -1;
            esp_task_wdt_reset();
            continue;
        } else if (len == 0) {
            ESP_LOGI(TAG, "TCP connection closed by peer, marking for reconnect...");
            is_tcp_connected = false;
            close(tcp_sock);
            tcp_sock = -1;
            esp_task_wdt_reset();
            continue;
        }

        // 优化 debug hex log：len > 128 时仅摘要，避免慢循环
        if (debug_mode && len <= 128) {
            char hex_buf[UART_BUF_SIZE * 3 + 1] = {0};  // +1 for null
            char *ptr = hex_buf;
            for (int i = 0; i < len; i++) {
                ptr += sprintf(ptr, "%02X ", rx_buffer[i]);  // sprintf 直接追加，无 strlen
            }
            ESP_LOGI(TAG, "Raw TCP receive buffer (hex): %s (length: %d)", hex_buf, len);
        } else if (debug_mode) {
            ESP_LOGI(TAG, "TCP packet received: %d bytes (hex log skipped)", len);
        }

        // Parse and process the received data
        esp_err_t result = parse_tcp_command(tcp_sock, rx_buffer, len);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "Failed to parse TCP command (len=%d)", len);
        }

        esp_task_wdt_reset();  // 每循环喂狗
    }

    free(rx_buffer);
    vTaskDelete(NULL);
}
static void uart_tcp_bridge_task(void *pvParameters) {
    esp_task_wdt_add(NULL);  // Register this task with WDT
    char *rx_buffer = malloc(UART_BUF_SIZE);  // Move malloc outside if-block
    if (!rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffers");
        vTaskDelete(NULL);
    }

    uint8_t uart_batch[UART_BUF_SIZE];  // Only used in !USB_CDC
    size_t batch_len = 0;
    uint32_t last_flush = 0;

    while (1) {
        // UART read + batching ONLY in non-USB mode
        if (!USB_CDC) {
            int len = uart_read_bytes(UART_NUM_0, uart_batch + batch_len, sizeof(uart_batch) - batch_len - 1, UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
            if (len > 0) {
                batch_len += len;
                uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if (batch_len >= FLUSH_BATCH_SIZE || (now - last_flush >= FLUSH_INTERVAL_MS)) {
                    if (tcp_sock != -1 && is_tcp_connected) {
                        esp_err_t send_result = tcp_send_data(tcp_sock, (char*)uart_batch, batch_len, rx_buffer, UART_BUF_SIZE, false);
                        if (send_result == ESP_ERR_TIMEOUT) {
                            ESP_LOGW(TAG, "Send timeout, skipping this batch but continuing (no drop, len=%u; will retry next)", (unsigned)batch_len);
                        } else if (send_result != ESP_OK) {
                            ESP_LOGW(TAG, "Fatal send error, closing socket and marking for reconnect (len=%u, err=%s)", 
                                    (unsigned)batch_len, esp_err_to_name(send_result));
                            is_tcp_connected = false;
                            if (tcp_sock != -1) {
                                close(tcp_sock);
                                tcp_sock = -1;
                            }
                        } else if (debug_mode) {
                            ESP_LOGI(TAG, "Flushed %zu bytes to TCP (real-time)", batch_len);
                        }
                        batch_len = 0;
                        last_flush = now;
                    } else {
                        ESP_LOGE(TAG, "TCP not connected, dropping batched data (len=%u)", (unsigned)batch_len);
                        batch_len = 0;
                    }
                }
            }
        }

        // Shared TCP/WiFi management (runs in both modes)
        if (!is_wifi_connected()) {
            if (debug_mode) ESP_LOGW(TAG, "WiFi not connected, closing socket");
            if (tcp_sock != -1) {
                close(tcp_sock);
                tcp_sock = -1;
                is_tcp_connected = false;
            }
            if (!USB_CDC) batch_len = 0;  // Clear only if UART mode
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_task_wdt_reset();
            continue;
        }

        // TCP connect/reconnect logic (full implementation)
        if (is_tcp_connected && tcp_sock != -1) {
            vTaskDelay(pdMS_TO_TICKS(1));  // Healthy connection
        } else if (tcp_sock == -1 && is_wifi_connected()) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last_reconnect_time < tcp_reconnect_delay_ms) {
                vTaskDelay(pdMS_TO_TICKS(1));  // Short yield if backoff active
                esp_task_wdt_reset();
                continue;
            }

            struct sockaddr_in dest_addr;
            dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(PORT);

            tcp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
            if (tcp_sock < 0) {
                ESP_LOGE(TAG, "Unable to create TCP socket: errno %d", errno);
                last_reconnect_time = now;
                tcp_reconnect_delay_ms = (tcp_reconnect_delay_ms * 2 > TCP_RETRY_MAX_DELAY_MS) ? TCP_RETRY_MAX_DELAY_MS : tcp_reconnect_delay_ms * 2;
                vTaskDelay(pdMS_TO_TICKS(tcp_reconnect_delay_ms));
                esp_task_wdt_reset();
                continue;
            }

            struct timeval timeout;
            timeout.tv_sec = TCP_CONNECT_TIMEOUT_MS / 1000;
            timeout.tv_usec = (TCP_CONNECT_TIMEOUT_MS % 1000) * 1000;
            setsockopt(tcp_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
            setsockopt(tcp_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

            // Enhanced keepalive: Shorter idle (5s) with fewer probes (3) for ~8s detection
            int keepalive = 1;
            int keepidle = 5;      // 5s idle before probes
            int keepintvl = 1;     // 1s between probes
            int keepcnt = 3;       // 3 probes
            setsockopt(tcp_sock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
            setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
            setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
            setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt));
            ESP_LOGI(TAG, "TCP Keepalive optimized: idle=5s, probes=3x1s (total ~8s detection)");

            // Disable Nagle's algorithm for real-time small-packet sending
            int nodelay = 1;
            setsockopt(tcp_sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
            ESP_LOGI(TAG, "TCP_NODELAY enabled for real-time sending");

            // Set larger TCP send buffer (64kB)
            int send_buf_size = 65536;  // 64kB
            setsockopt(tcp_sock, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size));
            if (debug_mode) ESP_LOGI(TAG, "Set TCP SO_SNDBUF to 64kB");

            int err = connect(tcp_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err != 0) {
                ESP_LOGW(TAG, "TCP unable to connect to %s:%d: errno %d", HOST_IP_ADDR, PORT, errno);
                close(tcp_sock);
                tcp_sock = -1;
                is_tcp_connected = false;
                last_reconnect_time = now;
                tcp_reconnect_delay_ms = (tcp_reconnect_delay_ms * 2 > TCP_RETRY_MAX_DELAY_MS) ? TCP_RETRY_MAX_DELAY_MS : tcp_reconnect_delay_ms * 2;
                vTaskDelay(pdMS_TO_TICKS(tcp_reconnect_delay_ms));
                esp_task_wdt_reset();
                continue;
            }

            // Set non-blocking after connect
            int flags = fcntl(tcp_sock, F_GETFL, 0);
            if (flags < 0) {
                ESP_LOGE(TAG, "fcntl F_GETFL failed: errno %d", errno);
                close(tcp_sock);
                tcp_sock = -1;
                is_tcp_connected = false;
                last_reconnect_time = now;
                tcp_reconnect_delay_ms = (tcp_reconnect_delay_ms * 2 > TCP_RETRY_MAX_DELAY_MS) ? TCP_RETRY_MAX_DELAY_MS : tcp_reconnect_delay_ms * 2;
                vTaskDelay(pdMS_TO_TICKS(tcp_reconnect_delay_ms));
                esp_task_wdt_reset();
                continue;
            }
            if (fcntl(tcp_sock, F_SETFL, flags | O_NONBLOCK) < 0) {
                ESP_LOGE(TAG, "fcntl F_SETFL O_NONBLOCK failed: errno %d", errno);
                close(tcp_sock);
                tcp_sock = -1;
                is_tcp_connected = false;
                last_reconnect_time = now;
                tcp_reconnect_delay_ms = (tcp_reconnect_delay_ms * 2 > TCP_RETRY_MAX_DELAY_MS) ? TCP_RETRY_MAX_DELAY_MS : tcp_reconnect_delay_ms * 2;
                vTaskDelay(pdMS_TO_TICKS(tcp_reconnect_delay_ms));
                esp_task_wdt_reset();
                continue;
            }

            ESP_LOGI(TAG, "Successfully connected to %s:%d (backoff reset)", HOST_IP_ADDR, PORT);
            is_tcp_connected = true;
            tcp_reconnect_delay_ms = TCP_RETRY_BASE_DELAY_MS;  // Reset backoff on success
            gpio_set_level(LED_GPIO, 1); // Ensure LED on upon TCP connect
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));  // Wait if no WiFi
        }

        esp_task_wdt_reset();
    }

    // Cleanup
    if (tcp_sock != -1) {
        close(tcp_sock);
        tcp_sock = -1;
        is_tcp_connected = false;
    }
    free(rx_buffer);
    vTaskDelete(NULL);
}
// USB CDC callbacks
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
        ESP_LOGI(TAG, "USB CDC disconnected");
    } else {
        ESP_LOGI(TAG, "USB CDC connected");
    }
}

void tinyusb_cdc_line_coding_changed_callback(int itf, cdcacm_event_t *event) {
    // In USB mode, ignore (virtual baud), or log
    cdcacm_event_line_coding_changed_data_t *coding = &event->line_coding_changed_data;
    ESP_LOGI(TAG, "USB CDC line coding changed: baud=%lu, stop=%d, parity=%d, data=%d",
             coding->p_line_coding->bit_rate, coding->p_line_coding->stop_bits,
             coding->p_line_coding->parity, coding->p_line_coding->data_bits);
}

// USB to TCP task
static void usb_to_tcp_task(void *pvParameters) {
    esp_task_wdt_add(NULL);  // Register this task with WDT
    app_message_t msg;
    while (1) {
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY)) {
            if (tcp_sock != -1 && is_tcp_connected) {
                esp_err_t send_result = tcp_send_data(tcp_sock, (char*)msg.buf, msg.buf_len, NULL, 0, false);  // No response expected
                if (send_result != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to send USB data to TCP (len=%u)", (unsigned)msg.buf_len);
                }
            } else {
                ESP_LOGW(TAG, "TCP not connected, dropping USB data (len=%u)", (unsigned)msg.buf_len);
            }
        }
        esp_task_wdt_reset();
    }
}

/* WiFi event handler for ESP32 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGE(TAG, "WiFi disconnected: reason=%d (2=wrong pw, 15=no assoc, 201=beacon timeout)", event->reason);
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
        // Close sockets on disconnect
        if (tcp_sock != -1) {
            close(tcp_sock);
            tcp_sock = -1;
            is_tcp_connected = false;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(const char *ssid, const char *password)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);  // Changed to ANY_ID for broader IP events (e.g., lost IP)

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(82));

    // Static IP bypass for DHCP issues (server assigns 192.168.4.1, so use .100)
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_IF");
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 100);   // Static IP: 192.168.4.100
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);     // Gateway: AP IP
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);  // /24
    esp_netif_dhcpc_stop(sta_netif);  // Stop DHCP client
    esp_netif_set_ip_info(sta_netif, &ip_info);
    ESP_LOGI(TAG, "Static IP set: 192.168.4.100/24 gw 192.168.4.1 (DHCP bypassed)");

    // Manually set connected bit (simulate GOT_IP)
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    ESP_LOGI(TAG, "WiFi connected flag set manually");
}

// USB init
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

// USB device task
static void usb_device_task(void *param) {
    esp_task_wdt_add(NULL);  // Register this task with WDT
    vTaskDelay(pdMS_TO_TICKS(100));
    while (1) {
        tud_task();
        vTaskDelay(1);
        esp_task_wdt_reset();  // Feed WDT in loop
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        nvs_flash_erase();
        err = nvs_flash_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "NVS flash init failed: %s", esp_err_to_name(err));
            return;
        }
    }

    // FIX: Updated to new ESP-IDF v5+ WDT API (timeout in ms, config struct)
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 30000,  // 30 seconds
        .idle_core_mask = 0,  // No idle core monitoring
        .trigger_panic = true,
    };
    esp_task_wdt_init(&wdt_config);
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));  // Add current task (main) to WDT

    char ssid[32] = {0};
    char password[64] = {0};
    err = load_nvs_config(&current_baud_rate, &current_stop_bits, &current_parity, &current_data_bits, ssid, sizeof(ssid), password, sizeof(password), &debug_mode);
    if (err != ESP_OK) {
        strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, sizeof(ssid));
        strlcpy(password, DEFAULT_ESP_WIFI_PASS, sizeof(password));
        current_baud_rate = 115200;
        current_stop_bits = UART_STOP_BITS_1;
        current_parity = UART_PARITY_DISABLE;
        current_data_bits = UART_DATA_8_BITS;
        debug_mode = 0;
        esp_log_level_set(TAG, ESP_LOG_NONE);
    }

    // Note: boot_count handling removed; factory reset via button long press
    led_init();
    wifi_init_sta(ssid, password);
    authorcodeverify();
    uart_init(current_baud_rate, current_stop_bits, current_parity, current_data_bits);

    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 4, NULL);
    xTaskCreate(uart_tcp_bridge_task, "uart_tcp_bridge", 6144, NULL, 8, NULL);
    xTaskCreate(tcp_receive_task, "tcp_receive", 6144, NULL, 8, NULL);

    // USB CDC tasks if enabled
    if (USB_CDC) {
        size_t free_heap = esp_get_free_heap_size();
        ESP_LOGI(TAG, "Free heap before USB init: %zu bytes", free_heap);
        err = init_usb();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "USB init failed, falling back to UART mode");
        } else {
            app_queue = xQueueCreate(8, sizeof(app_message_t));  // Small queue
            if (!app_queue) ESP_LOGE(TAG, "Failed to create app_queue");
            xTaskCreate(usb_device_task, "usb_dev", 8192, NULL, 10, NULL);
            xTaskCreate(usb_to_tcp_task, "usb_to_tcp", 3072, NULL, 9, NULL);
            ESP_LOGI(TAG, "USB CDC tasks created");
        }
    }

    // Feed WDT in main
    while (1) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}