#ifndef _TUSB_COMPOSITE_MAIN_H_
#define _TUSB_COMPOSITE_MAIN_H_

#include <stdio.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define QUEUE_SIZE 50
#define CONFIG_TINYUSB_CDC_RX_BUFSIZE 512
#define DEFAULT_ESP_WIFI_SSID "FUNLIGHT"
#define DEFAULT_ESP_WIFI_PASS "funlight"
#define ESP_WIFI_CHANNEL 11
#define PORT 12345
#define UART_BUF_SIZE 512
#define CDC_FLUSH_TIMEOUT_MS 50
#define DEFAULT_BAUD_RATE 115200
#define LED_GPIO GPIO_NUM_17
#define BUTTON_GPIO GPIO_NUM_0
#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_CLICK_THRESHOLD_MS 500
#define BUTTON_LONG_PRESS_MS 5000
#define NVS_NAMESPACE "config"
#define NVS_KEY_SSID "wifi_ssid"
#define NVS_KEY_PASS "wifi_pass"

typedef struct {
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
    size_t buf_len;
    int itf;
} app_message_t;

#endif