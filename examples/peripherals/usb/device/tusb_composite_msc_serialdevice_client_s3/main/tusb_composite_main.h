#ifndef _TUSB_COMPOSITE_MAIN_H_
#define _TUSB_COMPOSITE_MAIN_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"  // For app_queue
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include <regex.h>
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "lwip/ip4_addr.h"
#include "esp_mac.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"


// Macro for mode selection (set to 0 for UART0 fallback)
#define USB_CDC 1  // 1: USB CDC mode (default), 0: UART0 mode

#define DEFAULT_ESP_WIFI_SSID      "FUNLIGHT"
#define DEFAULT_ESP_WIFI_PASS      "funlight"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10
#define NVS_NAMESPACE              "config"

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define HOST_IP_ADDR "192.168.4.1"
static const char *TAG = "slave";

#define PORT 12345
#define CONFIG_EXAMPLE_IPV4 1
#define UART_BUF_SIZE (512)
#define TCP_RETRY_BASE_DELAY_MS 1000  // Start with 1s
#define TCP_RETRY_MAX_DELAY_MS 32000  // Cap at 32s
#define TCP_CONNECT_TIMEOUT_MS 10000
#define TCP_KEEPALIVE_IDLE_MS 5000    // 5s idle before probes
#define RESPONSE_TIMEOUT_MS 5000
#define UART_READ_TIMEOUT_MS 1        // 1ms for real-time ~50B reads
#define FLUSH_BATCH_SIZE 100          // Flush if batch >=100B
#define FLUSH_INTERVAL_MS 5           // Or every 5ms

#define LED_GPIO GPIO_NUM_17
#define BUTTON_GPIO GPIO_NUM_0
#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_LONG_PRESS_MS 6000  // 6 seconds for factory reset

// USB CDC config
#define CONFIG_TINYUSB_CDC_RX_BUFSIZE UART_BUF_SIZE  // Reuse UART size

#endif