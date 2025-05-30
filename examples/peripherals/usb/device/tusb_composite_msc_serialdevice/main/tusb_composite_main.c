/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <errno.h>
#include <dirent.h>
#include <sys/stat.h>
#include <string.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <time.h>
#include <inttypes.h>
#include "esp_partition.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "driver/gpio.h"
#include "lwip/tcp.h"
#include "tinyusb.h"
#include "tusb_msc_storage.h"
#include "tusb_cdc_acm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_task_wdt.h"

#define BASE_PATH "/usb"
#define QUEUE_SIZE 50
#define RX_TIMEOUT_MS 1
#define FLUSH_TIMEOUT_MS 10
#define TEST_FILE_PATH "/usb/esp/test.txt"
#define DEFAULT_ESP_WIFI_SSID "FUNLIGHT"
#define DEFAULT_ESP_WIFI_PASS "funlight"
#define ESP_WIFI_CHANNEL 1
#define ESP_MAX_STA_CONN 4
#define PORT 12345
#define NVS_NAMESPACE "config"
#define NVS_KEY_SSID "wifi_ssid"
#define NVS_KEY_PASS "wifi_pass"
#define DEFAULT_BAUD_RATE 115200
#define UART_BUF_SIZE 512
#define MAX_CLIENTS 4
#define LED_PIN GPIO_NUM_17
#define BUTTON_PIN GPIO_NUM_0
#define LED_BLINK_PERIOD_MS 500
#define LED_BLINK_DURATION_MS 100
#define SEND_TIMEOUT_MS 50
#define RECV_TIMEOUT_MS 50
#define NOTIFICATION_QUEUE_SIZE 10
#define MAX_SEND_RETRIES 5
#define MIN_HEAP_THRESHOLD 10000
#define CONFIG_TINYUSB_CDC_RX_BUFSIZE 512
#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_CLICK_THRESHOLD_MS 500
#define BUTTON_LONG_PRESS_MS 5000
#define RESOURCE_CHECK_INTERVAL_MS 60000
#define CDC_WRITE_RETRIES 1
#define CDC_WRITE_RETRY_DELAY_MS 50
#define WATCHDOG_TIMEOUT_S 10
#define CDC_FLUSH_TIMEOUT_MS 50

static const char *TAG = "example_main";
static uint8_t cdc_rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
static QueueHandle_t app_queue;
static QueueHandle_t notification_queue;
static uint32_t last_baudrate = 0;
static uint32_t current_baud_rate = DEFAULT_BAUD_RATE;
static uint8_t debug_mode = 0; // Enabled for debugging
static char log_buffer[256];
static SemaphoreHandle_t client_mutex = NULL;
static int connected_clients = 0;
static uint64_t total_bytes_sent = 0;
static uint64_t total_bytes_queued = 0;
static bool tcp_send_paused = false;
static char cached_log_prefix[64] = "[00:00:00.000][device 0]";
static bool factory_state = true;
static char current_ssid[32] = DEFAULT_ESP_WIFI_SSID;
static char current_password[64] = DEFAULT_ESP_WIFI_PASS;
static char pending_ssid[32] = "";
static char pending_password[64] = "";
static bool led_blink_flag = false;
static uint32_t last_sent_baud_rate = 0;
static uint8_t last_sent_stop_bits = 0;
static uint8_t last_sent_parity = 0;
static uint8_t last_sent_data_bits = 0;

typedef struct {
    uint8_t mac[6];
    char ip[16];
    bool valid;
} client_mac_map_t;

static client_mac_map_t client_mac_map[ESP_MAX_STA_CONN] = {0};

typedef struct {
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
    size_t buf_len;
    uint8_t itf;
} app_message_t;

typedef struct {
    int sock;
    char ip[16];
    uint8_t mac[6];
    bool active;
    TickType_t last_recv_time;
    uint64_t bytes_sent;
} client_t;

typedef struct {
    char message[128];
} notification_t;

static client_t clients[MAX_CLIENTS];

#define CDC_LOG(isdebug, format, ...) do { \
    if ((isdebug) && !debug_mode) break; \
    if (esp_get_free_heap_size() != NULL && esp_get_free_heap_size() < MIN_HEAP_THRESHOLD) { \
        printf("Critical low heap: %zu, restarting\n", esp_get_free_heap_size()); \
        esp_restart(); \
    } \
    snprintf(log_buffer, sizeof(log_buffer), format, ##__VA_ARGS__); \
    size_t len = strlen(log_buffer); \
    size_t queued = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (uint8_t *)log_buffer, len); \
    if (queued == len) { \
        const char newline = '\n'; \
        tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (uint8_t *)&newline, 1); \
    } \
    esp_err_t flush_ret = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(100)); \
    if (flush_ret != ESP_OK || queued != len) { \
        if (!(isdebug)) { \
            printf("CDC write failed: %.*s\n", (int)len, log_buffer); \
        } \
    } \
} while (0)

static bool is_cdc_connected(void) {
    bool connected = tud_cdc_n_connected(TINYUSB_CDC_ACM_0);
    CDC_LOG(1, "CDC connection check: tud_cdc_n_connected=%d, tud_inited=%d\n", 
            connected, tud_inited());
    return tud_inited();
}

static void update_log_prefix(const char *client_ip)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm_info = gmtime(&tv.tv_sec);
    const char *last_octet = strrchr(client_ip, '.') ? strrchr(client_ip, '.') + 1 : "0";
    snprintf(cached_log_prefix, sizeof(cached_log_prefix), "[%02d:%02d:%02d.%03d][device %s]",
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec, (int)(tv.tv_usec / 1000), last_octet);
}

void tinyusb_cdc_line_state_cb(tinyusb_cdcacm_itf_t itf, cdcacm_event_t *event)
{
    CDC_LOG(1, "CDC line state: itf=%d, DTR=%d, RTS=%d, connected=%d, tud_inited=%d, free heap: %zu\n",
            itf, event->line_state_changed_data.dtr, event->line_state_changed_data.rts,
            tud_cdc_n_connected(TINYUSB_CDC_ACM_0), tud_inited(), esp_get_free_heap_size());
    if (event->line_state_changed_data.dtr && event->line_state_changed_data.rts) {
        CDC_LOG(1, "CDC connected, DTR=1, RTS=1\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    } else {
        CDC_LOG(1, "CDC disconnected, DTR=%d, RTS=%d\n",
                event->line_state_changed_data.dtr, event->line_state_changed_data.rts);
    }
}

static void write_baudrate_to_file(uint32_t baudrate, uint8_t stop_bits, uint8_t parity, uint8_t data_bits)
{
    if (baudrate == last_baudrate) {
        CDC_LOG(1, "Baudrate unchanged: %lu, skipping file write\n", baudrate);
        return;
    }
    if (tinyusb_msc_storage_in_use_by_usb_host()) {
        CDC_LOG(1, "MSC in use by USB host, skipping file write\n");
        return;
    }
    FILE *f = fopen(TEST_FILE_PATH, "a");
    if (f == NULL) {
        CDC_LOG(1, "Failed to open %s for appending: %s\n", TEST_FILE_PATH, strerror(errno));
        return;
    }
    fprintf(f, "Baudrate: %lu, StopBits: %u, Parity: %u, DataBits: %u\n",
            baudrate, stop_bits, parity, data_bits);
    fclose(f);
    last_baudrate = baudrate;
    CDC_LOG(1, "Wrote baudrate info to %s: %lu\n", TEST_FILE_PATH, baudrate);
}

static void send_to_client(client_t *client, const char *data, size_t len, const char *log_prefix)
{
    if (!client->active || client->sock < 0) {
        CDC_LOG(1, "%s Skipping send to inactive client %s\n", log_prefix, client->ip);
        return;
    }
    size_t free_heap = esp_get_free_heap_size();
    CDC_LOG(1, "%s Attempting to send %zu bytes to %s, free heap: %zu\n", log_prefix, len, client->ip, free_heap);
    int total_sent = 0, retries = MAX_SEND_RETRIES, delay_ms = 10;
    while (total_sent < len && retries > 0) {
        int sent = send(client->sock, data + total_sent, len - total_sent, 0);
        if (sent < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                CDC_LOG(1, "%s Send retry (%d left): errno %d (%s), free heap: %zu\n",
                        log_prefix, retries, errno, strerror(errno), free_heap);
                retries--;
                vTaskDelay(delay_ms / portTICK_PERIOD_MS);
                delay_ms *= 2;
                continue;
            }
            CDC_LOG(1, "%s Fatal send error: errno %d (%s)\n", log_prefix, errno, strerror(errno));
            shutdown(client->sock, SHUT_RDWR);
            close(client->sock);
            client->active = false;
            if (xSemaphoreTake(client_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                connected_clients--;
                tcp_send_paused = false;
                xSemaphoreGive(client_mutex);
            }
            CDC_LOG(1, "%s Client %s disconnected due to send error\n", log_prefix, client->ip);
            break;
        }
        total_sent += sent;
        client->bytes_sent += sent;
        total_bytes_sent += sent;
        delay_ms = 10;
        CDC_LOG(1, "%s Sent %d bytes to %s (total %d/%zu, client total: %" PRIu64 ", global total: %" PRIu64 ")\n",
                log_prefix, sent, client->ip, total_sent, len, client->bytes_sent, total_bytes_sent);
    }
    tcp_send_paused = (total_sent < len);
}

static void send_notification_to_clients(const char *message)
{
    notification_t notification;
    snprintf(notification.message, sizeof(notification.message), "%s", message);
    if (xQueueSend(notification_queue, &notification, pdMS_TO_TICKS(10)) != pdTRUE) {
        CDC_LOG(1, "Failed to queue notification\n");
    }
}

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    size_t free_heap = esp_get_free_heap_size();
    if (free_heap < MIN_HEAP_THRESHOLD) {
        CDC_LOG(1, "Critical low heap in CDC callback: %zu, restarting\n", free_heap);
        esp_restart();
    }
    if (uxQueueSpacesAvailable(app_queue) == 0) {
        CDC_LOG(1, "Queue full, skipping CDC read\n");
        return;
    }
    if (tcp_send_paused) {
        CDC_LOG(1, "TCP send paused, skipping CDC read\n");
        vTaskDelay(5 / portTICK_PERIOD_MS);
        return;
    }
    size_t rx_size = 0;
    esp_err_t ret = tinyusb_cdcacm_read(itf, cdc_rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK && rx_size > 0) {
        total_bytes_queued += rx_size;
        CDC_LOG(1, "Received %zu bytes on itf %d (total queued: %" PRIu64 ", free heap: %zu)\n",
                rx_size, itf, total_bytes_queued, free_heap);
        app_message_t tx_msg = { .buf_len = rx_size, .itf = itf };
        memcpy(tx_msg.buf, cdc_rx_buf, rx_size);
        if (xQueueSend(app_queue, &tx_msg, pdMS_TO_TICKS(2)) != pdTRUE) {
            CDC_LOG(1, "Queue full, dropped %zu bytes\n", rx_size);
            total_bytes_queued -= rx_size;
        }
    } else {
        CDC_LOG(1, "Read error: ret=%d, rx_size=%zu\n", ret, rx_size);
    }
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    CDC_LOG(1, "Line state changed on itf %d: DTR=%d, RTS=%d, free heap: %zu\n",
            itf, event->line_state_changed_data.dtr, event->line_state_changed_data.rts, esp_get_free_heap_size());
}

void tinyusb_cdc_line_coding_changed_callback(int itf, cdcacm_event_t *event)
{
    cdcacm_event_line_coding_changed_data_t *coding = &event->line_coding_changed_data;
    CDC_LOG(1, "Line coding changed on itf %d: baud=%ld, stop_bits=%d, parity=%d, data_bits=%d, free heap: %zu\n",
            itf, coding->p_line_coding->bit_rate, coding->p_line_coding->stop_bits,
            coding->p_line_coding->parity, coding->p_line_coding->data_bits, esp_get_free_heap_size());
    current_baud_rate = coding->p_line_coding->bit_rate;
    
    if (coding->p_line_coding->bit_rate != last_sent_baud_rate ||
        coding->p_line_coding->stop_bits != last_sent_stop_bits ||
        coding->p_line_coding->parity != last_sent_parity ||
        coding->p_line_coding->data_bits != last_sent_data_bits) {
        last_sent_baud_rate = coding->p_line_coding->bit_rate;
        last_sent_stop_bits = coding->p_line_coding->stop_bits;
        last_sent_parity = coding->p_line_coding->parity;
        last_sent_data_bits = coding->p_line_coding->data_bits;
        
        char message[128];
        snprintf(message, sizeof(message), "BAUD=%lu,STOPBIT=%d,PARITY=%d,DATABIT=%d\r\n", 
                coding->p_line_coding->bit_rate, coding->p_line_coding->stop_bits,
                coding->p_line_coding->parity, coding->p_line_coding->data_bits);
        send_notification_to_clients(message);
        CDC_LOG(1, "Serial parameters changed, notification sent: %s", message);
        write_baudrate_to_file(coding->p_line_coding->bit_rate, coding->p_line_coding->stop_bits,
                               coding->p_line_coding->parity, coding->p_line_coding->data_bits);
    } else {
        CDC_LOG(1, "Serial parameters unchanged, skipping notification\n");
    }
    
    if (debug_mode) {
        cdc_line_coding_t line_coding;
        tud_cdc_n_get_line_coding(itf, &line_coding);
        if (line_coding.bit_rate > 0) {
            CDC_LOG(1, "Verified line coding: baud=%ld, stop_bits=%d, parity=%d, data_bits=%d\n",
                    line_coding.bit_rate, line_coding.stop_bits, line_coding.parity, line_coding.data_bits);
        } else {
            CDC_LOG(1, "Invalid line coding received\n");
        }
    }
}

static bool file_exists(const char *file_path)
{
    struct stat buffer;
    return stat(file_path, &buffer) == 0;
}

static void file_operations(void)
{
    static bool file_initialized = false;
    if (file_initialized) return;
    if (tinyusb_msc_storage_in_use_by_usb_host()) {
        CDC_LOG(1, "MSC in use by USB host, skipping file operations\n");
        return;
    }
    const char *directory = "/usb/esp";
    const char *file_path = TEST_FILE_PATH;
    struct stat s = {0};
    if (stat(directory, &s) != 0) {
        if (mkdir(directory, 0775) != 0) {
            CDC_LOG(1, "mkdir failed with errno: %s\n", strerror(errno));
        }
    }
    if (!file_exists(file_path)) {
        CDC_LOG(1, "Creating file %s\n", file_path);
        FILE *f = fopen(file_path, "w");
        if (f == NULL) {
            CDC_LOG(1, "Failed to open %s for writing: %s\n", file_path, strerror(errno));
            return;
        }
        fprintf(f, "Hello World!\n");
        fclose(f);
    }
    file_initialized = true;
}

static esp_err_t init_storage(wl_handle_t *wl_handle)
{
    CDC_LOG(1, "Initializing storage, free heap: %zu\n", esp_get_free_heap_size());
    const esp_partition_t *data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, NULL);
    if (data_partition == NULL) {
        CDC_LOG(1, "Failed to find FATFS partition\n");
        return ESP_ERR_NOT_FOUND;
    }
    esp_err_t ret = wl_mount(data_partition, wl_handle);
    if (ret != ESP_OK) {
        CDC_LOG(1, "Wear levelling mount failed: %s\n", esp_err_to_name(ret));
        return ret;
    }
    const tinyusb_msc_spiflash_config_t config_spi = { .wl_handle = *wl_handle };
    ret = tinyusb_msc_storage_init_spiflash(&config_spi);
    if (ret != ESP_OK) {
        CDC_LOG(1, "MSC storage init failed: %s\n", esp_err_to_name(ret));
        return ret;
    }
    ret = tinyusb_msc_storage_mount(BASE_PATH);
    if (ret != ESP_OK) {
        CDC_LOG(1, "MSC storage mount failed: %s\n", esp_err_to_name(ret));
        return ret;
    }
    file_operations();
    return ESP_OK;
}

static void led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(LED_PIN, 0);
    CDC_LOG(1, "LED initialized to OFF\n");
}

static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static esp_err_t load_wifi_credentials(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        CDC_LOG(1, "Failed to open NVS namespace: %s\n", esp_err_to_name(ret));
        return ret;
    }

    size_t ssid_len = sizeof(current_ssid);
    ret = nvs_get_str(nvs_handle, NVS_KEY_SSID, current_ssid, &ssid_len);
    if (ret != ESP_OK) {
        CDC_LOG(1, "No SSID found in NVS, using default: %s\n", DEFAULT_ESP_WIFI_SSID);
        strlcpy(current_ssid, DEFAULT_ESP_WIFI_SSID, sizeof(current_ssid));
        factory_state = true;
    } else {
        factory_state = false;
    }

    size_t pass_len = sizeof(current_password);
    ret = nvs_get_str(nvs_handle, NVS_KEY_PASS, current_password, &pass_len);
    if (ret != ESP_OK) {
        CDC_LOG(1, "No password found in NVS, using default: %s\n", DEFAULT_ESP_WIFI_PASS);
        strlcpy(current_password, DEFAULT_ESP_WIFI_PASS, sizeof(current_password));
        factory_state = true;
    } else {
        factory_state = factory_state && false;
    }

    nvs_close(nvs_handle);
    CDC_LOG(1, "Loaded WiFi credentials: SSID=%s, Password=%s, Factory State=%d\n",
            current_ssid, current_password, factory_state);
    return ESP_OK;
}

static esp_err_t save_wifi_credentials(const char *ssid, const char *password)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        CDC_LOG(1, "Failed to open NVS namespace for writing: %s\n", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_str(nvs_handle, NVS_KEY_SSID, ssid);
    if (ret != ESP_OK) {
        CDC_LOG(1, "Failed to save SSID to NVS: %s\n", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    ret = nvs_set_str(nvs_handle, NVS_KEY_PASS, password);
    if (ret != ESP_OK) {
        CDC_LOG(1, "Failed to save password to NVS: %s\n", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        CDC_LOG(1, "Failed to commit NVS changes: %s\n", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    nvs_close(nvs_handle);
    CDC_LOG(1, "Saved WiFi credentials to NVS: SSID=%s, Password=%s\n", ssid, password);
    return ESP_OK;
}

static esp_err_t clear_nvs_config(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        CDC_LOG(1, "Failed to open NVS namespace for clearing: %s\n", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_erase_key(nvs_handle, NVS_KEY_SSID);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        CDC_LOG(1, "Failed to erase SSID from NVS: %s\n", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    ret = nvs_erase_key(nvs_handle, NVS_KEY_PASS);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        CDC_LOG(1, "Failed to erase password from NVS: %s\n", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        CDC_LOG(1, "Failed to commit NVS changes after clearing: %s\n", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    nvs_close(nvs_handle);
    CDC_LOG(1, "NVS configuration cleared successfully\n");
    return ESP_OK;
}

static void update_wifi_config(const char *ssid, const char *password)
{
    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(ssid),
            .channel = ESP_WIFI_CHANNEL,
            .authmode = strlen(password) == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
            .max_connection = ESP_MAX_STA_CONN,
            .beacon_interval = 100,
            .pmf_cfg = { .capable = true, .required = false },
            .ftm_responder = false
        },
    };
    strlcpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
    strlcpy((char *)wifi_config.ap.password, password, sizeof(wifi_config.ap.password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    CDC_LOG(1, "Updated WiFi config to SSID:%s, password:%s\n", ssid, password);
}

static void button_task(void *pvParameters)
{
    TickType_t last_press_time = 0;
    bool last_state = true;
    esp_task_wdt_add(NULL);
    while (1) {
        esp_task_wdt_reset();
        bool current_state = gpio_get_level(BUTTON_PIN);
        if (last_state && !current_state) {
            last_press_time = xTaskGetTickCount();
        } else if (!last_state && current_state) {
            TickType_t press_duration = xTaskGetTickCount() - last_press_time;
            if (press_duration < pdMS_TO_TICKS(BUTTON_CLICK_THRESHOLD_MS) && factory_state && connected_clients > 0) {
                uint8_t mac[6];
                char new_ssid[32];
                char new_password[64];
                char mac_str[13];
                esp_wifi_get_mac(WIFI_IF_AP, mac);
                snprintf(mac_str, sizeof(mac_str), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                snprintf(new_ssid, sizeof(new_ssid), "FUNLIGHT-%s", &mac_str[8]);
                snprintf(new_password, sizeof(new_password), "funlight-%s", &mac_str[8]);
                update_log_prefix("0.0.0.0");
                CDC_LOG(1, "%s Button click detected, updating WiFi SSID:%s, password:%s\n",
                        cached_log_prefix, new_ssid, new_password);
                char message[128];
                snprintf(message, sizeof(message), "WIFI=%s,%s\r\n", new_ssid, new_password);
                send_notification_to_clients(message);
                esp_err_t ret = save_wifi_credentials(new_ssid, new_password);
                if (ret == ESP_OK) {
                    update_wifi_config(new_ssid, new_password);
                    strlcpy(current_ssid, new_ssid, sizeof(current_ssid));
                    strlcpy(current_password, new_password, sizeof(current_password));
                    factory_state = false;
                    CDC_LOG(1, "%s WiFi credentials saved, restarting in 1000ms\n", cached_log_prefix);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                } else {
                    CDC_LOG(1, "%s Failed to save WiFi credentials, not restarting\n", cached_log_prefix);
                }
            }
        } else if (!last_state && !current_state) {
            TickType_t press_duration = xTaskGetTickCount() - last_press_time;
            if (press_duration >= pdMS_TO_TICKS(BUTTON_LONG_PRESS_MS)) {
                update_log_prefix("0.0.0.0");
                CDC_LOG(1, "%s Long press detected, clearing NVS configuration\n", cached_log_prefix);
                esp_err_t ret = clear_nvs_config();
                if (ret == ESP_OK) {
                    CDC_LOG(1, "%s NVS cleared, restarting device\n", cached_log_prefix);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                } else {
                    CDC_LOG(1, "%s Failed to clear NVS, not restarting\n", cached_log_prefix);
                }
            }
        }
        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
    }
}

static void led_task(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    while (1) {
        esp_task_wdt_reset();
        if (xSemaphoreTake(client_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (led_blink_flag) {
                gpio_set_level(LED_PIN, 1);
                CDC_LOG(1, "LED blink ON due to client data\n");
                vTaskDelay(pdMS_TO_TICKS(LED_BLINK_DURATION_MS));
                gpio_set_level(LED_PIN, 0);
                CDC_LOG(1, "LED blink OFF\n");
                led_blink_flag = false;
            } else {
                gpio_set_level(LED_PIN, connected_clients > 0 ? 1 : 0);
            }
            xSemaphoreGive(client_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_PERIOD_MS));
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    char message[128];
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_AP_STACONNECTED) {
            wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
            snprintf(message, sizeof(message), "Station %02x:%02x:%02x:%02x:%02x:%02x joined, AID=%d\n",
                     event->mac[0], event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5], event->aid);
            CDC_LOG(1, "%s", message);
            send_notification_to_clients(message);
            esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
            esp_netif_ip_info_t ip_info;
            esp_netif_get_ip_info(netif, &ip_info);
            char client_ip[16];
            struct sockaddr_in addr;
            socklen_t addr_len = sizeof(addr);
            for (int i = 0; i < ESP_MAX_STA_CONN; i++) {
                if (!client_mac_map[i].valid) {
                    memcpy(client_mac_map[i].mac, event->mac, 6);
                    client_mac_map[i].valid = true;
                    CDC_LOG(1, "Stored MAC %02x:%02x:%02x:%02x:%02x:%02x for client %d\n",
                            event->mac[0], event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5], i);
                    break;
                }
            }
        } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
            wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
            snprintf(message, sizeof(message), "Station %02x:%02x:%02x:%02x:%02x:%02x left, AID=%d\n",
                     event->mac[0], event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5], event->aid);
            CDC_LOG(1, "%s", message);
            send_notification_to_clients(message);
            for (int i = 0; i < ESP_MAX_STA_CONN; i++) {
                if (client_mac_map[i].valid && memcmp(client_mac_map[i].mac, event->mac, 6) == 0) {
                    client_mac_map[i].valid = false;
                    memset(client_mac_map[i].mac, 0, 6);
                    memset(client_mac_map[i].ip, 0, sizeof(client_mac_map[i].ip));
                    CDC_LOG(1, "Cleared MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                            event->mac[0], event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5]);
                    break;
                }
            }
        }
    }
}

static void setup_socket_options(int sock)
{
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    int nodelay = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    int sndbuf = 1024;
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    int rcvbuf = 512;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    struct timeval timeout = { .tv_sec = RECV_TIMEOUT_MS / 1000, .tv_usec = (RECV_TIMEOUT_MS % 1000) * 1000 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    int keepalive = 1;
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
}

static int create_listen_socket(const char *log_prefix)
{
    struct sockaddr_in dest_addr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_family = AF_INET,
        .sin_port = htons(PORT),
    };
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        CDC_LOG(0, "%s: Socket creation failed: %s\n", log_prefix, strerror(errno));
        return -1;
    }
    setup_socket_options(listen_sock);
    if (bind(listen_sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr)) != 0) {
        CDC_LOG(0, "%s: Socket bind failed: %s\n", log_prefix, strerror(errno));
        close(listen_sock);
        return -1;
    }
    if (listen(listen_sock, MAX_CLIENTS) != 0) {
        CDC_LOG(0, "%s: Socket listen failed: %s\n", log_prefix, strerror(errno));
        close(listen_sock);
        return -1;
    }
    CDC_LOG(1, "Socket listening on port %d, free heap: %zu\n", PORT, esp_get_free_heap_size());
    return listen_sock;
}

static bool get_client_mac(const char *client_ip, uint8_t *mac)
{
    for (int i = 0; i < ESP_MAX_STA_CONN; i++) {
        if (client_mac_map[i].valid && strcmp(client_ip, client_mac_map[i].ip) == 0) {
            memcpy(mac, client_mac_map[i].mac, 6);
            return true;
        }
    }
    return false;
}

static int accept_client(int listen_sock, char *addr_str, uint8_t *mac, const char *log_prefix)
{
    struct sockaddr_in source_addr;
    socklen_t addr_len = sizeof(source_addr);
    int sock = accept(listen_sock, (struct sockaddr*)&source_addr, &addr_len);
    if (sock < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            CDC_LOG(1, "%s: Accept failed: %s\n", log_prefix, strerror(errno));
        }
        return -1;
    }
    inet_ntoa_r(source_addr.sin_addr, addr_str, 16);
    setup_socket_options(sock);
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    for (int i = 0; i < ESP_MAX_STA_CONN; i++) {
        if (client_mac_map[i].valid) {
            client_mac_map[i].ip[0] = '\0';
        }
    }
    for (int i = 0; i < ESP_MAX_STA_CONN; i++) {
        if (client_mac_map[i].valid && client_mac_map[i].ip[0] == '\0') {
            strlcpy(client_mac_map[i].ip, addr_str, sizeof(client_mac_map[i].ip));
            break;
        }
    }
    if (!get_client_mac(addr_str, mac)) {
        CDC_LOG(1, "%s No MAC found for client IP %s\n", log_prefix, addr_str);
        memset(mac, 0, 6);
    }
    CDC_LOG(1, "%s Client connected: %s, MAC %02x:%02x:%02x:%02x:%02x:%02x, free heap: %zu\n",
            log_prefix, addr_str, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], esp_get_free_heap_size());
    char welcome_msg[128];
    snprintf(welcome_msg, sizeof(welcome_msg), "Welcome! Current baud rate: %lu\n", current_baud_rate);
    send(sock, welcome_msg, strlen(welcome_msg), 0);
    return sock;
}

static int add_client(int sock, const char *addr_str, const uint8_t *mac, const char *log_prefix)
{
    int client_idx = -1;
    if (xSemaphoreTake(client_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i].active && memcmp(clients[i].mac, mac, 6) == 0) {
                CDC_LOG(1, "%s Replacing existing client with MAC %02x:%02x:%02x:%02x:%02x:%02x at index %d\n",
                        log_prefix, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], i);
                shutdown(clients[i].sock, SHUT_RDWR);
                close(clients[i].sock);
                clients[i].sock = sock;
                strlcpy(clients[i].ip, addr_str, sizeof(clients[i].ip));
                clients[i].last_recv_time = xTaskGetTickCount();
                clients[i].bytes_sent = 0;
                tcp_send_paused = false;
                client_idx = i;
                break;
            }
        }
        if (client_idx == -1) {
            for (int i = 0; i < MAX_CLIENTS; i++) {
                if (!clients[i].active) {
                    client_idx = i;
                    clients[i].sock = sock;
                    clients[i].active = true;
                    strlcpy(clients[i].ip, addr_str, sizeof(clients[i].ip));
                    memcpy(clients[i].mac, mac, 6);
                    clients[i].last_recv_time = xTaskGetTickCount();
                    clients[i].bytes_sent = 0;
                    connected_clients++;
                    break;
                }
            }
        }
        xSemaphoreGive(client_mutex);
    }
    if (client_idx == -1) {
        CDC_LOG(0, "%s Max clients reached, rejecting connection\n", log_prefix);
        shutdown(sock, SHUT_RDWR);
        close(sock);
        return -1;
    }
    CDC_LOG(1, "%s Added client %s, MAC %02x:%02x:%02x:%02x:%02x:%02x, total clients: %d, free heap: %zu\n",
            log_prefix, addr_str, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], connected_clients, esp_get_free_heap_size());
    return client_idx;
}

static void wifi_init_softap(const char *ssid, const char *password)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    update_wifi_config(ssid, password);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));
    ESP_ERROR_CHECK(esp_wifi_start());
    if (debug_mode) {
        CDC_LOG(1, "SoftAP started. SSID:%s password:%s channel:%d, free heap: %zu\n",
                ssid, password, ESP_WIFI_CHANNEL, esp_get_free_heap_size());
    }
}

static void disconnect_client(int sock, int client_idx, const char *log_prefix)
{
    if (xSemaphoreTake(client_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (sock != -1 && clients[client_idx].active) {
            CDC_LOG(1, "%s Disconnecting client %s, MAC %02x:%02x:%02x:%02x:%02x:%02x, bytes sent: %" PRIu64 ", free heap: %zu\n",
                    log_prefix, clients[client_idx].ip,
                    clients[client_idx].mac[0], clients[client_idx].mac[1], clients[client_idx].mac[2],
                    clients[client_idx].mac[3], clients[client_idx].mac[4], clients[client_idx].mac[5],
                    clients[client_idx].bytes_sent, esp_get_free_heap_size());
            shutdown(sock, SHUT_RDWR);
            close(sock);
            clients[client_idx].active = false;
            clients[client_idx].sock = -1;
            connected_clients--;
            tcp_send_paused = false;
            CDC_LOG(1, "%s Client disconnected, remaining clients: %d\n", log_prefix, connected_clients);
        }
        xSemaphoreGive(client_mutex);
    }
}

static void tcp_server_task(void *pvParameters)
{
    static char rx_buffer[UART_BUF_SIZE];
    char addr_str[16];
    uint8_t client_mac[6];
    for (int i = 0; i < MAX_CLIENTS; i++) {
        clients[i].sock = -1;
        clients[i].active = false;
        clients[i].last_recv_time = 0;
        clients[i].bytes_sent = 0;
        memset(clients[i].mac, 0, 6);
    }
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
    while (1) {
        esp_task_wdt_reset();
        update_log_prefix("0.0.0.0");
        int listen_sock = create_listen_socket(cached_log_prefix);
        if (listen_sock < 0) {
            CDC_LOG(0, "%s Failed to create listen socket, retrying\n", cached_log_prefix);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        CDC_LOG(1, "%s Listening on port %d, free heap: %zu\n", cached_log_prefix, PORT, esp_get_free_heap_size());
        while (1) {
            esp_task_wdt_reset();
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(listen_sock, &read_fds);
            int max_fd = listen_sock;
            int active_sockets = 1;
            if (xSemaphoreTake(client_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                CDC_LOG(1, "%s Active clients:", cached_log_prefix);
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (clients[i].active) {
                        CDC_LOG(1, " [%d]=%s(sock=%d)", i, clients[i].ip, clients[i].sock);
                        FD_SET(clients[i].sock, &read_fds);
                        if (clients[i].sock > max_fd) max_fd = clients[i].sock;
                        active_sockets++;
                    }
                }
                CDC_LOG(1, "\n");
                xSemaphoreGive(client_mutex);
            }
            CDC_LOG(1, "%s select: monitoring %d sockets (listen=%d, max_fd=%d)\n",
                    cached_log_prefix, active_sockets, listen_sock, max_fd);
            struct timeval select_timeout = { .tv_sec = 0, .tv_usec = 50000 };
            int ready = select(max_fd + 1, &read_fds, NULL, NULL, &select_timeout);
            if (ready < 0) {
                CDC_LOG(0, "%s select failed: %s\n", cached_log_prefix, strerror(errno));
                break;
            }
            CDC_LOG(1, "%s select returned %d ready sockets\n", cached_log_prefix, ready);
            if (ready == 0) {
                notification_t notification;
                if (xQueueReceive(notification_queue, &notification, 0)) {
                    if (xSemaphoreTake(client_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        for (int i = 0; i < MAX_CLIENTS; i++) {
                            if (clients[i].active) {
                                update_log_prefix(clients[i].ip);
                                CDC_LOG(1, "%s Sending notification: %s\n", cached_log_prefix, notification.message);
                                send_to_client(&clients[i], notification.message, strlen(notification.message), cached_log_prefix);
                            }
                        }
                        xSemaphoreGive(client_mutex);
                    }
                }
                continue;
            }
            if (FD_ISSET(listen_sock, &read_fds)) {
                CDC_LOG(1, "%s Listen socket readable, accepting client\n", cached_log_prefix);
                int sock = accept_client(listen_sock, addr_str, client_mac, cached_log_prefix);
                if (sock >= 0) {
                    int client_idx = add_client(sock, addr_str, client_mac, cached_log_prefix);
                    if (client_idx < 0) {
                        CDC_LOG(0, "%s Failed to add client %s, closing\n", cached_log_prefix, addr_str);
                        close(sock);
                    } else {
                        CDC_LOG(1, "%s Client %s connected, socket %d, MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                                cached_log_prefix, addr_str, sock,
                                client_mac[0], client_mac[1], client_mac[2],
                                client_mac[3], client_mac[4], client_mac[5]);
                    }
                }
            }
            if (xSemaphoreTake(client_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (clients[i].active && FD_ISSET(clients[i].sock, &read_fds)) {
                        CDC_LOG(1, "%s Socket %d for client %s is readable\n", cached_log_prefix, clients[i].sock, clients[i].ip);
                        int state;
                        socklen_t sock_len = sizeof(state);
                        getsockopt(clients[i].sock, SOL_SOCKET, SO_ERROR, &state, &sock_len);
                        CDC_LOG(1, "%s Socket %d state: %d\n", cached_log_prefix, clients[i].sock, state);
                        memset(rx_buffer, 0, UART_BUF_SIZE);
                        int len = recv(clients[i].sock, rx_buffer, UART_BUF_SIZE - 1, 0);
                        if (len < 0) {
                            CDC_LOG(2, "%s recv failed for client %s: %s\n", cached_log_prefix, clients[i].ip, strerror(errno));
                            disconnect_client(clients[i].sock, i, cached_log_prefix);
                            continue;
                        } else if (len == 0) {
                            CDC_LOG(2, "%s Client %s disconnected\n", cached_log_prefix, clients[i].ip);
                            disconnect_client(clients[i].sock, i, cached_log_prefix);
                            continue;
                        } else {
                            if (len >= UART_BUF_SIZE) {
                                CDC_LOG(0, "%s Received data too large (%d bytes) for buffer size %d, truncating\n", 
                                        cached_log_prefix, len, UART_BUF_SIZE);
                                len = UART_BUF_SIZE - 1;
                            }
                            clients[i].last_recv_time = xTaskGetTickCount();
                            led_blink_flag = true;
                            CDC_LOG(1, "%s Received %d bytes from client %s at tick %u: ", 
                                    cached_log_prefix, len, clients[i].ip, (unsigned)xTaskGetTickCount());
                            for (int j = 0; j < len; j++) {
                                CDC_LOG(1, "0x%02x ", (unsigned char)rx_buffer[j]);
                            }
                            CDC_LOG(1, "\n");
                            bool cdc_connected = is_cdc_connected();
                            CDC_LOG(1, "%s CDC connection status: %d\n", cached_log_prefix, cdc_connected);
                            size_t queued = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (uint8_t *)rx_buffer, len);
                            // if (queued == len) {
                            //     const char newline = '\n';
                            //     tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (uint8_t *)&newline, 1);
                            // }
                            esp_err_t flush_ret = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(100));
                            if (flush_ret != ESP_OK || queued != len) {
                                CDC_LOG(0, "%s CDC write failed for client %s, queued: %zu of %d bytes, flush: %s\n", 
                                        cached_log_prefix, clients[i].ip, queued, len, esp_err_to_name(flush_ret));
                                printf("%.*s\n", len, rx_buffer);
                            } else {
                                CDC_LOG(1, "%s Successfully wrote %d bytes to CDC for client %s\n", 
                                        cached_log_prefix, len, clients[i].ip);
                            }
                        }
                    }
                }
                xSemaphoreGive(client_mutex);
            }
            esp_task_wdt_reset();
        }
        close(listen_sock);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static esp_err_t init_system(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    size_t free_heap = esp_get_free_heap_size();
    CDC_LOG(1, "Starting USB composite with WiFi and TCP server, free heap: %zu\n", free_heap);
    if (free_heap < MIN_HEAP_THRESHOLD) {
        CDC_LOG(0, "Initial heap too low: %zu, restarting\n", free_heap);
        esp_restart();
    }
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = load_wifi_credentials();
    if (ret != ESP_OK) {
        CDC_LOG(1, "Failed to load WiFi credentials, using defaults\n");
    }

    esp_log_level_set(TAG, debug_mode ? ESP_LOG_INFO : ESP_LOG_ERROR);
    esp_log_level_set("TinyUSB", ESP_LOG_ERROR);
    app_queue = xQueueCreate(QUEUE_SIZE, sizeof(app_message_t));
    notification_queue = xQueueCreate(NOTIFICATION_QUEUE_SIZE, sizeof(notification_t));
    client_mutex = xSemaphoreCreateMutex();
    if (!app_queue || !notification_queue || !client_mutex) {
        CDC_LOG(0, "Failed to create queue or mutex\n");
        esp_restart();
    }
    wl_handle_t wl_handle;
    ret = init_storage(&wl_handle);
    if (ret != ESP_OK) return ret;
    wifi_init_softap(current_ssid, current_password);
    led_init();
    button_init();
    xTaskCreate(led_task, "led_task", 2048, NULL, 4, NULL);
    xTaskCreate(tcp_server_task, "tcp_server_task", 8192, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
    return ESP_OK;
}

static esp_err_t init_usb(void)
{
    CDC_LOG(1, "USB Composite initialization\n");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .string_descriptor_count = 0,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };
    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        CDC_LOG(0, "TinyUSB driver install failed: %s\n", esp_err_to_name(ret));
        return ret;
    }
    CDC_LOG(1, "TinyUSB driver installed, tud_inited=%d\n", tud_inited());
    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = CONFIG_TINYUSB_CDC_RX_BUFSIZE,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &tinyusb_cdc_line_state_changed_callback,
        .callback_line_coding_changed = &tinyusb_cdc_line_coding_changed_callback
    };
    ret = tusb_cdc_acm_init(&acm_cfg);
    if (ret != ESP_OK) {
        CDC_LOG(0, "CDC ACM init failed: %s\n", esp_err_to_name(ret));
        return ret;
    }
    CDC_LOG(1, "CDC ACM initialized, connected=%d\n", tud_cdc_n_connected(TINYUSB_CDC_ACM_0));
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret = init_system();
    if (ret != ESP_OK) {
        CDC_LOG(0, "System initialization failed: %s\n", esp_err_to_name(ret));
        esp_restart();
    }
    ret = init_usb();
    if (ret != ESP_OK) {
        CDC_LOG(0, "USB initialization failed: %s\n", esp_err_to_name(ret));
        esp_restart();
    }
    CDC_LOG(1, "USB Composite with WiFi and TCP server initialization DONE, free heap: %zu\n", esp_get_free_heap_size());
    app_message_t msg;
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
    while (1) {
        esp_task_wdt_reset();
        size_t free_heap = esp_get_free_heap_size();
        if (free_heap < MIN_HEAP_THRESHOLD) {
            CDC_LOG(0, "Low heap in main loop: %zu, restarting\n", free_heap);
            esp_restart();
        }
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY)) {
            update_log_prefix("0.0.0.0");
            CDC_LOG(1, "Processing queued message from itf %d, len=%zu\n", msg.itf, msg.buf_len);
            if (xSemaphoreTake(client_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (connected_clients == 0) {
                    CDC_LOG(1, "%s No clients connected, skipping TCP forward\n", cached_log_prefix);
                }
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (clients[i].active) {
                        update_log_prefix(clients[i].ip);
                        CDC_LOG(1, "%s Forwarding %zu bytes to client %s, MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                                cached_log_prefix, msg.buf_len, clients[i].ip,
                                clients[i].mac[0], clients[i].mac[1], clients[i].mac[2],
                                clients[i].mac[3], clients[i].mac[4], clients[i].mac[5]);
                        send_to_client(&clients[i], (const char *)msg.buf, msg.buf_len, cached_log_prefix);
                    }
                }
                xSemaphoreGive(client_mutex);
            } else {
                CDC_LOG(1, "%s Failed to acquire client mutex\n", cached_log_prefix);
            }
        }
        esp_task_wdt_reset();
    }
}