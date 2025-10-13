/*
 * Optimized TCP Server with USB-CDC Console, Multi-Client, NVS, LED, and Commands for ESP32-S2
 * - SoftAP Wi-Fi (SSID: FUNLIGHT, Password: funlight by default)
 * - Multiple TCP clients (up to 4) on port 12345
 * - Console input sent to all clients, client data printed to console
 * - NVS for baud_rate, ssid, password, debug_mode
 * - LED: Blinks when no clients, solid when clients connected
 * - Commands: BAUD, WIFI, DEBUG, RSTDEFAULT (only before transparent data)
 * - Transparent mode: After first transparent data, all console data is passthrough
 * - Optimized for high-throughput serial-to-TCP forwarding
 * - Added: Automatic baud rate detection and client notifications
 *
 * SPDX-FileCopyrightText: 2025 xAI
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include <stdio.h>
#include <sys/param.h>
#include <sys/time.h>
#include <time.h>
#include <ctype.h>
#include <inttypes.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "driver/gpio.h"
#include <lwip/tcp.h>
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#define DEFAULT_ESP_WIFI_SSID      "FUNLIGHT"
#define DEFAULT_ESP_WIFI_PASS      "funlight"
#define ESP_WIFI_CHANNEL           1
#define ESP_MAX_STA_CONN           4
#define PORT                       12345
#define NVS_NAMESPACE              "config"
#define DEFAULT_BAUD_RATE          115200
#define UART_BUF_SIZE              1024
#define MAX_CLIENTS                4
#define BAUD_RESPONSE_TIMEOUT_MS   2000
#define BIND_RETRY_DELAY_MS        1000
#define LED_PIN                    GPIO_NUM_2
#define LED_BLINK_PERIOD_MS        500
#define SEND_TIMEOUT_MS            500
#define RECV_TIMEOUT_MS            50
#define NOTIFICATION_QUEUE_SIZE    10

static const char *TAG = "tcp_server";

static SemaphoreHandle_t client_count_mutex = NULL;
static SemaphoreHandle_t clients_mutex = NULL;
static QueueHandle_t notification_queue = NULL;
static uint32_t current_baud_rate = DEFAULT_BAUD_RATE;
static uint8_t debug_mode = 0;  // Force enabled for troubleshooting
static int connected_clients = 0;
static bool transparent_mode = false;
static bool transparent_data_detected = false;

typedef struct {
    int sock;
    char ip[16];
    bool active;
    bool responded;
    TickType_t last_recv_time;
} client_t;

static client_t clients[MAX_CLIENTS];

// Structure for notifications to send to clients
typedef struct {
    char message[128];
} notification_t;

static void print_raw_log(const char *data)
{
    if (data) {
        printf("%s", data);
        fflush(stdout);  // Ensure immediate output
    }
}

static void print_hex_dump(const char *data, int len) {
    if (!debug_mode) return;
    char hex_buf[256];
    char ascii_buf[64];
    int hex_len = 0, ascii_len = 0;
    for (int i = 0; i < len && hex_len < sizeof(hex_buf) - 4 && ascii_len < sizeof(ascii_buf) - 2; i++) {
        hex_len += snprintf(hex_buf + hex_len, sizeof(hex_buf) - hex_len, "%02X ", (unsigned char)data[i]);
        ascii_len += snprintf(ascii_buf + ascii_len, sizeof(ascii_buf) - ascii_len, "%c", isprint((unsigned char)data[i]) ? data[i] : '.');
    }
    ESP_LOGI(TAG, "Raw data hex: %s | ASCII: %s", hex_buf, ascii_buf);
}

static void led_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(LED_PIN, 1);
}

static void led_task(void *pvParameters) {
    bool led_state = false;
    while (1) {
        if (xSemaphoreTake(client_count_mutex, portMAX_DELAY) == pdTRUE) {
            gpio_set_level(LED_PIN, connected_clients > 0 ? 0 : (led_state = !led_state) ? 0 : 1);
            xSemaphoreGive(client_count_mutex);
        }
        vTaskDelay(LED_BLINK_PERIOD_MS / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void save_nvs_config(uint32_t baud_rate, const char *ssid, const char *password, uint8_t debug) {
    nvs_handle_t nvs_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) != ESP_OK) return;
    if (ssid) nvs_set_str(nvs_handle, "wifi_ssid", ssid);
    if (password) nvs_set_str(nvs_handle, "wifi_pass", password);
    nvs_set_u32(nvs_handle, "baud_rate", baud_rate);
    nvs_set_u8(nvs_handle, "debug_mode", debug);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}

static void reset_to_defaults(void) {
    nvs_handle_t nvs_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) != ESP_OK) return;
    nvs_erase_all(nvs_handle);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    esp_restart();
}

static void get_log_prefix(char *prefix, size_t prefix_len, const char *client_ip) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm_info = gmtime(&tv.tv_sec);
    char *last_octet = strrchr(client_ip, '.') ? strrchr(client_ip, '.') + 1 : "0";
    snprintf(prefix, prefix_len, "[%02d:%02d:%02d.%03d][device %s]", 
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec, (int)(tv.tv_usec / 1000), last_octet);
}

static bool has_connected_clients(void) {
    bool result = false;
    if (xSemaphoreTake(client_count_mutex, portMAX_DELAY) == pdTRUE) {
        result = connected_clients > 0;
        xSemaphoreGive(client_count_mutex);
    }
    return result;
}

static void send_to_client(client_t *client, const char *data, size_t len, char *log_prefix) {
    int total_sent = 0, retries = 5;
    while (total_sent < len && retries > 0) {
        int sent = send(client->sock, data + total_sent, len - total_sent, 0);
        if (sent < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                if (debug_mode) ESP_LOGW(TAG, "%s Send retry (%d left): errno %d (%s)", log_prefix, retries, errno, strerror(errno));
                retries--;
                vTaskDelay(5 / portTICK_PERIOD_MS);
                continue;
            }
            if (debug_mode) ESP_LOGE(TAG, "%s Fatal send error: errno %d (%s)", log_prefix, errno, strerror(errno));
            shutdown(client->sock, SHUT_RDWR);
            close(client->sock);
            client->active = false;
            client->responded = false;
            if (xSemaphoreTake(client_count_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                connected_clients--;
                xSemaphoreGive(client_count_mutex);
            }
            break;
        }
        total_sent += sent;
        if (debug_mode) ESP_LOGI(TAG, "%s Sent %d bytes (total %d/%d)", log_prefix, sent, total_sent, len);
    }
}

static void send_notification_to_clients(const char *message)
{
    notification_t notification;
    snprintf(notification.message, sizeof(notification.message), "%s", message);
    if (xQueueSend(notification_queue, &notification, pdMS_TO_TICKS(10)) != pdTRUE) {
        print_raw_log("Failed to queue notification\n");
    }
}

// Task to handle sending notifications to clients
static void notification_task(void *pvParameters)
{
    notification_t notification;
    char log_prefix[64];
    while (1) {
        if (xQueueReceive(notification_queue, &notification, portMAX_DELAY)) {
            if (xSemaphoreTake(clients_mutex, portMAX_DELAY) == pdTRUE) {
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (clients[i].active) {
                        get_log_prefix(log_prefix, sizeof(log_prefix), clients[i].ip);
                        send_to_client(&clients[i], notification.message, strlen(notification.message), log_prefix);
                    }
                }
                xSemaphoreGive(clients_mutex);
            }
        }
    }
}

static bool broadcast_and_wait(const char *data, size_t len, int sender_sock) {
    char log_prefix[64];
    if (xSemaphoreTake(clients_mutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i].active && clients[i].sock != sender_sock) {
                clients[i].responded = false;
                get_log_prefix(log_prefix, sizeof(log_prefix), clients[i].ip);
                send_to_client(&clients[i], data, len, log_prefix);
            }
        }
        xSemaphoreGive(clients_mutex);
    }

    int timeout_ms = BAUD_RESPONSE_TIMEOUT_MS;
    bool all_responded = false;
    while (timeout_ms > 0) {
        all_responded = true;
        if (xSemaphoreTake(clients_mutex, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < MAX_CLIENTS; i++) {
                if (clients[i].active && clients[i].sock != sender_sock && !clients[i].responded) {
                    all_responded = false;
                    break;
                }
            }
            xSemaphoreGive(clients_mutex);
        }
        if (all_responded) break;
        vTaskDelay(100 / portTICK_PERIOD_MS);
        timeout_ms -= 100;
    }
    return all_responded;
}

static void load_nvs_config(uint32_t *baud_rate, char *ssid, size_t ssid_len, char *password, size_t pass_len, uint8_t *debug) {
    nvs_handle_t nvs_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle) == ESP_OK) {
        size_t len = ssid_len;
        if (nvs_get_str(nvs_handle, "wifi_ssid", ssid, &len) != ESP_OK) strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, ssid_len);
        len = pass_len;
        if (nvs_get_str(nvs_handle, "wifi_pass", password, &len) != ESP_OK) strlcpy(password, DEFAULT_ESP_WIFI_PASS, pass_len);
        if (nvs_get_u32(nvs_handle, "baud_rate", baud_rate) != ESP_OK) *baud_rate = DEFAULT_BAUD_RATE;
        if (nvs_get_u8(nvs_handle, "debug_mode", debug) != ESP_OK) *debug = 1;
        nvs_close(nvs_handle);
    } else {
        *baud_rate = DEFAULT_BAUD_RATE;
        strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, ssid_len);
        strlcpy(password, DEFAULT_ESP_WIFI_PASS, pass_len);
        *debug = 1;
    }
    if (strlen(ssid) == 0 || strlen(ssid) > 32) strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, ssid_len);
    if (strlen(password) > 0 && (strlen(password) < 8 || strlen(password) > 64)) strlcpy(password, DEFAULT_ESP_WIFI_PASS, pass_len);
    esp_log_level_set(TAG, *debug ? ESP_LOG_INFO : ESP_LOG_NONE);
}

static void process_baud_command(const char *cmd, char *log_prefix) {
    uint32_t new_baud = atoi(cmd + 5);
    if (new_baud >= 9600 && new_baud <= 921600) {
        if (debug_mode) ESP_LOGI(TAG, "%s Changing baud rate to %" PRIu32, log_prefix, new_baud);
        current_baud_rate = new_baud;
        save_nvs_config(new_baud, NULL, NULL, debug_mode);
        printf("OK\r\n");
        // Notify clients about the baud rate change
        char message[128];
        snprintf(message, sizeof(message), "Baud rate changed to: %lu\n", current_baud_rate);
        send_notification_to_clients(message);
    } else {
        if (debug_mode) ESP_LOGE(TAG, "%s Invalid baud rate: %s", log_prefix, cmd + 5);
        printf("Invalid baud rate\r\n");
    }
    fflush(stdout);
}

static void process_wifi_command(const char *cmd, char *log_prefix) {
    const char *ssid = cmd + 5;
    char *password = strchr(ssid, ',');
    if (!password) {
        if (debug_mode) ESP_LOGE(TAG, "%s Invalid WIFI command format", log_prefix);
        printf("Invalid WIFI command format\r\n");
        fflush(stdout);
        return;
    }
    *password = 0;
    password++;
    char *end = strchr(password, '\r');
    if (end) *end = 0;
    if (strlen(ssid) == 0 || strlen(ssid) > 32 || !isprint((unsigned char)ssid[0])) {
        if (debug_mode) ESP_LOGE(TAG, "%s Invalid SSID: %s", log_prefix, ssid);
        printf("Invalid SSID\r\n");
    } else if (strlen(password) > 0 && (strlen(password) < 8 || strlen(password) > 64 || !isprint((unsigned char)password[0]))) {
        if (debug_mode) ESP_LOGE(TAG, "%s Invalid password length or characters", log_prefix);
        printf("Invalid password\r\n");
    } else {
        if (debug_mode) ESP_LOGI(TAG, "%s WiFi configured: SSID=%s, restarting...", log_prefix, ssid);
        save_nvs_config(current_baud_rate, ssid, password, debug_mode);
        printf("OK\r\n");
        fflush(stdout);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
    }
    fflush(stdout);
}

static void process_debug_command(const char *cmd, char *log_prefix) {
    uint8_t new_debug = atoi(cmd + 6);
    if (new_debug == 0 || new_debug == 1) {
        if (debug_mode) ESP_LOGI(TAG, "%s Changing debug mode to %u", log_prefix, new_debug);
        debug_mode = new_debug;
        esp_log_level_set(TAG, debug_mode ? ESP_LOG_INFO : ESP_LOG_NONE);
        save_nvs_config(current_baud_rate, NULL, NULL, debug_mode);
        printf("OK\r\n");
    } else {
        if (debug_mode) ESP_LOGE(TAG, "%s Invalid debug mode: %s", log_prefix, cmd + 6);
        printf("Invalid debug mode\r\n");
    }
    fflush(stdout);
}

static void process_console_command(char *rx_buffer, int rx_pos, char *log_prefix) {
    rx_buffer[rx_pos - 1] = 0;
    if (rx_pos <= 1) return;
    if (strncmp(rx_buffer, "BAUD=", 5) == 0) {
        process_baud_command(rx_buffer, log_prefix);
    } else if (strncmp(rx_buffer, "WIFI=", 5) == 0) {
        process_wifi_command(rx_buffer, log_prefix);
    } else if (strncmp(rx_buffer, "DEBUG=", 6) == 0) {
        process_debug_command(rx_buffer, log_prefix);
    } else if (strncmp(rx_buffer, "RSTDEFAULT", 10) == 0) {
        if (debug_mode) ESP_LOGI(TAG, "%s Factory reset triggered", log_prefix);
        reset_to_defaults();
    }
}

static void broadcast_console_data(const char *data, size_t len, char *log_prefix) {
    if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int j = 0; j < MAX_CLIENTS; j++) {
            if (clients[j].active && clients[j].sock != -1) {
                send_to_client(&clients[j], data, len, log_prefix);
            }
        }
        xSemaphoreGive(clients_mutex);
    }
}

static void console_task(void *pvParameters) {
    char rx_buffer[UART_BUF_SIZE];
    char send_buffer[UART_BUF_SIZE];
    char log_prefix[64];
    int rx_pos = 0, send_pos = 0;
    TickType_t last_flush_time = 0;

    while (1) {
        char temp_buffer[128];
        size_t bytes_read = fread(temp_buffer, 1, sizeof(temp_buffer), stdin);
        TickType_t current_time = xTaskGetTickCount();

        if (bytes_read > 0) {
            get_log_prefix(log_prefix, sizeof(log_prefix), "0.0.0.0");

            if (transparent_data_detected) {
                transparent_mode = true;
                if (send_pos + bytes_read < UART_BUF_SIZE) {
                    memcpy(send_buffer + send_pos, temp_buffer, bytes_read);
                    send_pos += bytes_read;
                }
            } else {
                for (size_t i = 0; i < bytes_read; i++) {
                    char c = temp_buffer[i];
                    if (rx_pos < UART_BUF_SIZE - 1) {
                        rx_buffer[rx_pos++] = c;
                        rx_buffer[rx_pos] = 0;
                        send_buffer[send_pos++] = c;
                        if (send_pos >= UART_BUF_SIZE || c == '\n' || c == '\r') {
                            if (!transparent_mode && !has_connected_clients() && (c == '\n' || c == '\r')) {
                                process_console_command(rx_buffer, rx_pos, log_prefix);
                                rx_pos = 0;
                                send_pos = 0;
                                continue;
                            }
                            transparent_mode = true;
                            transparent_data_detected = true;
                            broadcast_console_data(send_buffer, send_pos, log_prefix);
                            send_pos = 0;
                        }
                    } else {
                        memmove(rx_buffer, rx_buffer + 1, rx_pos);
                        rx_buffer[rx_pos - 1] = c;
                        rx_buffer[rx_pos] = 0;
                        send_pos = 0;
                    }
                }
            }
        }

        if (send_pos > 0 && (current_time - last_flush_time) >= pdMS_TO_TICKS(30)) {
            transparent_mode = true;
            transparent_data_detected = true;
            broadcast_console_data(send_buffer, send_pos, log_prefix);
            send_pos = 0;
            last_flush_time = current_time;
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void setup_socket_options(int sock) {
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    int nodelay = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    int sndbuf = 8192;
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    struct timeval timeout = { .tv_sec = SEND_TIMEOUT_MS / 1000, .tv_usec = (SEND_TIMEOUT_MS % 1000) * 1000 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    timeout.tv_sec = RECV_TIMEOUT_MS / 1000;
    timeout.tv_usec = (RECV_TIMEOUT_MS % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

static int create_listen_socket(char *log_prefix) {
    struct sockaddr_in dest_addr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_family = AF_INET,
        .sin_port = htons(PORT)
    };
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        printf("%s Socket creation failed: errno %d (%s)\n", log_prefix, errno, strerror(errno));
        return -1;
    }
    setup_socket_options(listen_sock);
    if (bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        printf("%s Socket bind failed: errno %d (%s)\n", log_prefix, errno, strerror(errno));
        close(listen_sock);
        return -1;
    }
    if (listen(listen_sock, MAX_CLIENTS) != 0) {
        printf("%s Socket listen failed: errno %d (%s)\n", log_prefix, errno, strerror(errno));
        close(listen_sock);
        return -1;
    }
    if (debug_mode) ESP_LOGI(TAG, "Socket listening on port %d", PORT);
    return listen_sock;
}

static int accept_client(int listen_sock, char *addr_str, char *log_prefix) {
    struct sockaddr_in source_addr;
    socklen_t addr_len = sizeof(source_addr);
    int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
    if (sock < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            printf("%s Accept failed: errno %d (%s)\n", log_prefix, errno, strerror(errno));
        }
        return -1;
    }
    inet_ntoa_r(source_addr.sin_addr, addr_str, 128);
    setup_socket_options(sock);
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    printf("%s Client connected: %s\n", log_prefix, addr_str);
    // Send welcome message with current baud rate
    char welcome_msg[128];
    snprintf(welcome_msg, sizeof(welcome_msg), "Welcome! Current baud rate: %lu\n", current_baud_rate);
    send(sock, welcome_msg, strlen(welcome_msg), 0);
    return sock;
}

static int add_client(int sock, const char *addr_str, char *log_prefix) {
    int client_idx = -1;
    if (xSemaphoreTake(clients_mutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (!clients[i].active) {
                client_idx = i;
                clients[i].sock = sock;
                clients[i].active = true;
                clients[i].responded = false;
                strlcpy(clients[i].ip, addr_str, sizeof(clients[i].ip));
                clients[i].last_recv_time = xTaskGetTickCount();
                break;
            }
        }
        xSemaphoreGive(clients_mutex);
    }
    if (client_idx == -1) {
        printf("%s Max clients reached, rejecting connection\n", log_prefix);
        shutdown(sock, SHUT_RDWR);
        close(sock);
        return -1;
    }
    if (xSemaphoreTake(client_count_mutex, portMAX_DELAY) == pdTRUE) {
        connected_clients++;
        xSemaphoreGive(client_count_mutex);
    }
    return client_idx;
}

static void process_client_baud(int sock, const char *rx_buffer, char *log_prefix) {
    uint32_t new_baud = atoi(rx_buffer + 5);
    if (new_baud >= 9600 && new_baud <= 921600) {
        if (debug_mode) ESP_LOGI(TAG, "%s Received client baud rate change request: %" PRIu32, log_prefix, new_baud);
        if (broadcast_and_wait(rx_buffer, strlen(rx_buffer), sock)) {
            if (debug_mode) ESP_LOGI(TAG, "%s Changing baud rate to %" PRIu32, log_prefix, new_baud);
            current_baud_rate = new_baud;
            save_nvs_config(new_baud, NULL, NULL, debug_mode);
            send(sock, "OK\r\n", 4, 0);
            // Notify clients about the baud rate change
            char message[128];
            snprintf(message, sizeof(message), "Baud rate changed to: %lu\n", current_baud_rate);
            send_notification_to_clients(message);
        } else {
            if (debug_mode) ESP_LOGE(TAG, "%s Not all clients responded, baud rate change aborted", log_prefix);
            send(sock, "Not all clients responded, baud rate change aborted\r\n", 52, 0);
        }
    } else {
        if (debug_mode) ESP_LOGE(TAG, "%s Invalid baud rate: %s", log_prefix, rx_buffer + 5);
        send(sock, "Invalid baud rate\r\n", 19, 0);
    }
}

static void process_client_wifi(int sock, char *rx_buffer, char *log_prefix) {
    const char *ssid = rx_buffer + 5;
    char *password = strchr(ssid, ',');
    if (!password) {
        if (debug_mode) ESP_LOGE(TAG, "%s Invalid WIFI command format", log_prefix);
        send(sock, "Invalid WIFI command format\r\n", 29, 0);
        return;
    }
    *password = 0;
    password++;
    char *end = strchr(password, '\r');
    if (end) *end = 0;
    if (strlen(ssid) == 0 || strlen(ssid) > 32 || !isprint((unsigned char)ssid[0])) {
        if (debug_mode) ESP_LOGE(TAG, "%s Invalid SSID: %s", log_prefix, ssid);
        send(sock, "Invalid SSID\r\n", 14, 0);
    } else if (strlen(password) > 0 && (strlen(password) < 8 || strlen(password) > 64 || !isprint((unsigned char)password[0]))) {
        if (debug_mode) ESP_LOGE(TAG, "%s Invalid password length or characters", log_prefix);
        send(sock, "Invalid password\r\n", 18, 0);
    } else if (broadcast_and_wait(rx_buffer, strlen(rx_buffer), sock)) {
        if (debug_mode) ESP_LOGI(TAG, "%s WiFi configured: SSID=%s, restarting...", log_prefix, ssid);
        save_nvs_config(current_baud_rate, ssid, password, debug_mode);
        send(sock, "OK\r\n", 4, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
    } else {
        if (debug_mode) ESP_LOGE(TAG, "%s Not all clients responded, WiFi configuration aborted", log_prefix);
        send(sock, "Not all clients responded, WiFi configuration aborted\r\n", 52, 0);
    }
}

static void process_client_debug(int sock, const char *rx_buffer, char *log_prefix) {
    uint8_t new_debug = atoi(rx_buffer + 6);
    if (new_debug == 0 || new_debug == 1) {
        if (debug_mode) ESP_LOGI(TAG, "%s Received client debug mode change request: %u", log_prefix, new_debug);
        if (broadcast_and_wait(rx_buffer, strlen(rx_buffer), sock)) {
            if (debug_mode) ESP_LOGI(TAG, "%s Changing debug mode to %u", log_prefix, new_debug);
            debug_mode = new_debug;
            esp_log_level_set(TAG, debug_mode ? ESP_LOG_INFO : ESP_LOG_NONE);
            save_nvs_config(current_baud_rate, NULL, NULL, debug_mode);
            send(sock, "OK\r\n", 4, 0);
        } else {
            if (debug_mode) ESP_LOGE(TAG, "%s Not all clients responded, debug mode change aborted", log_prefix);
            send(sock, "Not all clients responded, debug mode change aborted\r\n", 52, 0);
        }
    } else {
        if (debug_mode) ESP_LOGE(TAG, "%s Invalid debug mode: %s", log_prefix, rx_buffer + 6);
        send(sock, "Invalid debug mode\r\n", 20, 0);
    }
}

static void forward_client_data(int sender_sock, const char *data, size_t len, char *log_prefix) {
    if (xSemaphoreTake(clients_mutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i].active && clients[i].sock != sender_sock) {
                send_to_client(&clients[i], data, len, log_prefix);
            }
        }
        xSemaphoreGive(clients_mutex);
    }
}

static void handle_client_data(int sock, int client_idx, char *rx_buffer, int len, char *log_prefix, char *addr_str) {
    rx_buffer[len] = 0;
    char temp_buf[UART_BUF_SIZE + 256];
    snprintf(temp_buf, sizeof(temp_buf), "%s", rx_buffer);
    print_raw_log(temp_buf);

    if (strncmp(rx_buffer, "OK", 2) == 0 || strcmp(rx_buffer, "OK\r\n") == 0) {
        if (xSemaphoreTake(clients_mutex, portMAX_DELAY) == pdTRUE) {
            clients[client_idx].responded = true;
            xSemaphoreGive(clients_mutex);
        }
        if (debug_mode) ESP_LOGI(TAG, "%s Processed OK response", log_prefix);
        forward_client_data(sock, rx_buffer, len, log_prefix);
    } else if (strncmp(rx_buffer, "BAUD=", 5) == 0) {
        process_client_baud(sock, rx_buffer, log_prefix);
    } else if (strncmp(rx_buffer, "WIFI=", 5) == 0) {
        process_client_wifi(sock, rx_buffer, log_prefix);
    } else if (strncmp(rx_buffer, "DEBUG=", 6) == 0) {
        process_client_debug(sock, rx_buffer, log_prefix);
    } else if (strncmp(rx_buffer, "RSTDEFAULT", 10) == 0) {
        if (debug_mode) ESP_LOGI(TAG, "%s Factory reset triggered", log_prefix);
        forward_client_data(sock, rx_buffer, len, log_prefix);
        reset_to_defaults();
    } else {
        transparent_mode = true;
        transparent_data_detected = true;
    }
}

static void disconnect_client(int sock, int client_idx, char *log_prefix) {
    if (xSemaphoreTake(clients_mutex, portMAX_DELAY) == pdTRUE) {
        if (sock != -1) {
            shutdown(sock, SHUT_RDWR);
            close(sock);
            clients[client_idx].active = false;
            clients[client_idx].responded = false;
            if (xSemaphoreTake(client_count_mutex, portMAX_DELAY) == pdTRUE) {
                connected_clients--;
                xSemaphoreGive(client_count_mutex);
            }
            printf("%s Client disconnected, remaining clients: %d\n", log_prefix, connected_clients);
        }
        xSemaphoreGive(clients_mutex);
    }
}

// USB CDC callback for baud rate changes
void tinyusb_cdc_line_coding_changed_callback(int itf, cdcacm_event_t *event)
{
    cdcacm_event_line_coding_changed_data_t *coding = &event->line_coding_changed_data;
    uint32_t new_baudrate = coding->p_line_coding->bit_rate;

    print_raw_log("Line coding changed on itf ");
    char itf_str[16];
    snprintf(itf_str, sizeof(itf_str), "%d: baud=%ld, stop_bits=%d, parity=%d, data_bits=%d\n",
             itf, new_baudrate, coding->p_line_coding->stop_bits,
             coding->p_line_coding->parity, coding->p_line_coding->data_bits);
    print_raw_log(itf_str);

    // Check if baud rate has changed
    if (new_baudrate != current_baud_rate) {
        current_baud_rate = new_baudrate;
        save_nvs_config(current_baud_rate, NULL, NULL, debug_mode);

        // Notify clients about the baud rate change
        char message[128];
        snprintf(message, sizeof(message), "Baud rate changed to: %lu\n", current_baud_rate);
        send_notification_to_clients(message);
    }

    // Verify line coding
    cdc_line_coding_t line_coding;
    tud_cdc_n_get_line_coding(itf, &line_coding);
    if (line_coding.bit_rate > 0) {
        char verify_str[128];
        snprintf(verify_str, sizeof(verify_str), "Verified line coding: baud=%ld, stop_bits=%d, parity=%d, data_bits=%d\n",
                 line_coding.bit_rate, line_coding.stop_bits, line_coding.parity, line_coding.data_bits);
        print_raw_log(verify_str);
    } else {
        print_raw_log("Invalid line coding received\n");
    }
}

// Initialize USB CDC
static void usb_cdc_init(void)
{
    print_raw_log("Initializing USB CDC...\n");

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .string_descriptor_count = 0,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    print_raw_log("TinyUSB driver installed\n");

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = CONFIG_TINYUSB_CDC_RX_BUFSIZE,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = &tinyusb_cdc_line_coding_changed_callback
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    print_raw_log("CDC ACM initialized\n");

    if (tusb_cdc_acm_initialized(TINYUSB_CDC_ACM_0)) {
        print_raw_log("CDC ACM confirmed initialized\n");
    } else {
        print_raw_log("CDC ACM not initialized\n");
    }

    // Log initial line coding
    cdc_line_coding_t line_coding;
    tud_cdc_n_get_line_coding(TINYUSB_CDC_ACM_0, &line_coding);
    if (line_coding.bit_rate > 0) {
        current_baud_rate = line_coding.bit_rate;
        char initial_str[128];
        snprintf(initial_str, sizeof(initial_str), "Initial line coding: baud=%ld, stop_bits=%d, parity=%d, data_bits=%d\n",
                 line_coding.bit_rate, line_coding.stop_bits, line_coding.parity, line_coding.data_bits);
        print_raw_log(initial_str);
    } else {
        print_raw_log("Invalid initial line coding\n");
    }
}

// WiFi event handler for client connection/disconnection
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        char message[128];
        snprintf(message, sizeof(message), "Station "MACSTR" joined, AID=%d\n", MAC2STR(event->mac), event->aid);
        print_raw_log(message);
        send_notification_to_clients(message);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        char message[128];
        snprintf(message, sizeof(message), "Station "MACSTR" left, AID=%d, reason=%d\n", MAC2STR(event->mac), event->aid, event->reason);
        print_raw_log(message);
        send_notification_to_clients(message);
    }
}

static void wifi_init_softap(const char *ssid, const char *password) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(ssid),
            .channel = ESP_WIFI_CHANNEL,
            .authmode = strlen(password) == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
            .max_connection = ESP_MAX_STA_CONN,
            .beacon_interval = 100,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
            .ftm_responder = false
        },
    };
    strlcpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
    strlcpy((char *)wifi_config.ap.password, password, sizeof(wifi_config.ap.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));
    ESP_ERROR_CHECK(esp_wifi_start());

    if (debug_mode) ESP_LOGI(TAG, "SoftAP started. SSID:%s password:%s channel:%d", ssid, password, ESP_WIFI_CHANNEL);
}

static void tcp_server_task(void *pvParameters) {
    char rx_buffer[UART_BUF_SIZE], addr_str[128], log_prefix[64];
    TaskHandle_t console_task_handle = NULL;

    for (int i = 0; i < MAX_CLIENTS; i++) {
        clients[i].sock = -1;
        clients[i].active = false;
        clients[i].responded = false;
        clients[i].last_recv_time = 0;
    }

    while (1) {
        get_log_prefix(log_prefix, sizeof(log_prefix), "0.0.0.0");
        int listen_sock = create_listen_socket(log_prefix);
        if (listen_sock < 0) {
            vTaskDelay(BIND_RETRY_DELAY_MS / portTICK_PERIOD_MS);
            continue;
        }

        int flags = fcntl(listen_sock, F_GETFL, 0);
        fcntl(listen_sock, F_SETFL, flags | O_NONBLOCK);

        if (!console_task_handle && xTaskCreate(console_task, "console_task", 6144, NULL, 5, &console_task_handle) != pdPASS) {
            printf("%s Failed to create console_task\n", log_prefix);
            close(listen_sock);
            vTaskDelay(BIND_RETRY_DELAY_MS / portTICK_PERIOD_MS);
            continue;
        }
        if (debug_mode) ESP_LOGI(TAG, "Console task created successfully");

        while (1) {
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(listen_sock, &read_fds);
            int max_fd = listen_sock;

            if (xSemaphoreTake(clients_mutex, portMAX_DELAY) == pdTRUE) {
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (clients[i].active) {
                        FD_SET(clients[i].sock, &read_fds);
                        if (clients[i].sock > max_fd) max_fd = clients[i].sock;
                    }
                }
                xSemaphoreGive(clients_mutex);
            }

            struct timeval select_timeout = { .tv_sec = 1, .tv_usec = 0 };
            int ready = select(max_fd + 1, &read_fds, NULL, NULL, &select_timeout);
            if (ready < 0) {
                printf("%s Select failed: errno %d (%s)\n", log_prefix, errno, strerror(errno));
                break;
            } else if (ready == 0) {
                continue;
            }

            if (FD_ISSET(listen_sock, &read_fds)) {
                int sock = accept_client(listen_sock, addr_str, log_prefix);
                if (sock >= 0) {
                    int client_idx = add_client(sock, addr_str, log_prefix);
                    if (client_idx < 0) {
                        close(sock);
                    }
                }
            }

            if (xSemaphoreTake(clients_mutex, portMAX_DELAY) == pdTRUE) {
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (clients[i].active && FD_ISSET(clients[i].sock, &read_fds)) {
                        int len = recv(clients[i].sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
                        get_log_prefix(log_prefix, sizeof(log_prefix), clients[i].ip);
                        if (len < 0) {
                            if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                            ESP_LOGE(TAG, "%s Receive failed: errno %d (%s)", log_prefix, errno, strerror(errno));
                            disconnect_client(clients[i].sock, i, log_prefix);
                        } else if (len == 0) {
                            ESP_LOGI(TAG, "%s Client disconnected", log_prefix);
                            disconnect_client(clients[i].sock, i, log_prefix);
                        } else {
                            ESP_LOGI(TAG, "%s Received %d bytes from client %s", log_prefix, len, clients[i].ip);
                            handle_client_data(clients[i].sock, i, rx_buffer, len, log_prefix, clients[i].ip);
                        }
                    }
                }
                xSemaphoreGive(clients_mutex);
            }

            // Log stack high water mark periodically
            if (debug_mode) {
                UBaseType_t stack_high_water_mark = uxTaskGetStackHighWaterMark(NULL);
                ESP_LOGI(TAG, "tcp_server stack high water mark: %u bytes remaining", stack_high_water_mark * sizeof(StackType_t));
            }
        }

        close(listen_sock);
        vTaskDelay(BIND_RETRY_DELAY_MS / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("ESP32-S2 TCP Server Starting...\r\n");
    fflush(stdout);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (debug_mode) ESP_LOGI(TAG, "Free SRAM at startup: %u bytes", heap_caps_get_free_size(MALLOC_CAP_8BIT));

    char ssid[32] = {0}, password[64] = {0};
    load_nvs_config(&current_baud_rate, ssid, sizeof(ssid), password, sizeof(password), &debug_mode);
    esp_log_level_set(TAG, debug_mode ? ESP_LOG_INFO : ESP_LOG_NONE);

    client_count_mutex = xSemaphoreCreateMutex();
    clients_mutex = xSemaphoreCreateMutex();
    notification_queue = xQueueCreate(NOTIFICATION_QUEUE_SIZE, sizeof(notification_t));
    if (!client_count_mutex || !clients_mutex || !notification_queue) {
        printf("Failed to create mutex or queue\n");
        esp_restart();
    }

    // Initialize USB CDC for baud rate detection
    usb_cdc_init();

    led_init();
    wifi_init_softap(ssid, password);
    xTaskCreate(tcp_server_task, "tcp_server", 8192, NULL, 5, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 4, NULL);
    xTaskCreate(notification_task, "notification_task", 4096, NULL, 5, NULL);
}