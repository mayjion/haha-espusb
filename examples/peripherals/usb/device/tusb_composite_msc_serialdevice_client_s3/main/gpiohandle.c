#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "esp_system.h"  // For esp_restart()
#include "gpiohandle.h"
#include "wificonfig.h"  // For clear_nvs_config
#include "esp_wifi.h"    // For esp_wifi_get_mac
#include "tcp_client.h"  // For send_control_to_tcp and active_client
#include <stdio.h>       // For snprintf
#include <string.h>      // For strlen

static const char *TAG = "gpiohandle";
static EventGroupHandle_t led_event_group = NULL;  // For LED state signaling
static TaskHandle_t button_task_handle = NULL;  // For ISR notification
static bool factory_state = true;                // Track factory mode (initial without saved creds)

void led_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED GPIO config failed: %s", esp_err_to_name(ret));
        return;
    }
    gpio_set_level(LED_GPIO, 0);
    ESP_LOGI(TAG, "LED GPIO initialized");
}

// ISR handler for button (debounce in task via queue)
static void IRAM_ATTR button_isr_handler(void *arg) {
    BaseType_t high_task_wakeup = pdFALSE;
    xTaskNotifyFromISR(button_task_handle, 0, eNoAction, &high_task_wakeup);  // Wake task for debounce
    if (high_task_wakeup == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void button_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE | GPIO_INTR_NEGEDGE  // Interrupt on edge for efficiency
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_install_isr_service(0);  // Install global ISR service
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);  // Add ISR handler
    ESP_LOGI(TAG, "Button GPIO initialized with interrupts");
}


void button_task(void *pvParameters) {
    button_task_handle = xTaskGetCurrentTaskHandle();
    esp_task_wdt_add(NULL);

    TickType_t press_start = 0;
    bool pressed = false;
    uint32_t notify_value = 0;
    const TickType_t wdt_period = pdMS_TO_TICKS(5000);  // 5s max idle before reset

    while (1) {
        uint32_t notified_value;
        BaseType_t notified = xTaskNotifyWait(0, ULONG_MAX, &notified_value, wdt_period);
        esp_task_wdt_reset();  // Reset every wake (notify or timeout)

        if (notified == pdTRUE) {
            bool current_state = !gpio_get_level(BUTTON_GPIO);
            if (!pressed && current_state) {  // Press detected
                press_start = xTaskGetTickCount();
                pressed = true;
                ESP_LOGD(TAG, "Button pressed");
            } else if (pressed && !current_state) {  // Release
                TickType_t duration = xTaskGetTickCount() - press_start;
                uint32_t ms_duration = pdTICKS_TO_MS(duration);
                ESP_LOGI(TAG, "Button release detected (%ums)", ms_duration);
                if (ms_duration < BUTTON_CLICK_THRESHOLD_MS) {
                    // Client: Short click does nothing (no WiFi config send)
                    ESP_LOGI(TAG, "Button short click detected (%ums) - no action", ms_duration);
                } else if (ms_duration >= BUTTON_LONG_PRESS_MS) {  // >= for edge case
                    ESP_LOGW(TAG, "Button long press detected (%ums) - factory reset", ms_duration);
                    wifi_clear_config();
                    esp_restart();
                }
                pressed = false;
            }
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));  // Debounce
        }
        // No else: Timeout just resets WDT, loop continues
    }
}

EventGroupHandle_t get_led_event_group(void) {
    if (led_event_group == NULL) {
        led_event_group = xEventGroupCreate();
    }
    return led_event_group;
}

void led_task(void *pvParameters) {
    esp_task_wdt_add(NULL);  // Add to WDT

    TickType_t last_blink_toggle = 0;
    bool led_state = false;
    const TickType_t slow_delay = pdMS_TO_TICKS(500);  // Slow blink: WiFi disconnected
    const TickType_t fast_delay = pdMS_TO_TICKS(100);  // Fast blink: WiFi connected, TCP not
    const TickType_t check_delay = pdMS_TO_TICKS(100); // Check status every 100ms

    ESP_LOGI(TAG, "LED task started");

    static bool prev_wifi_connected = false;
    static bool prev_tcp_connected = false;
    static bool logged_slow = false;
    static bool logged_fast = false;
    static bool logged_on = false;

    while (1) {
        // Get WiFi and TCP status
        bool wifi_connected = false;
        wifi_ap_record_t ap_info;
        esp_err_t wifi_ret = esp_wifi_sta_get_ap_info(&ap_info);
        wifi_connected = (wifi_ret == ESP_OK);

        bool tcp_connected = false;
        if (client_mutex != NULL) {
            if (xSemaphoreTake(client_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                tcp_connected = active_client.active;
                xSemaphoreGive(client_mutex);
            }
        }

        TickType_t now = xTaskGetTickCount();
        bool toggled = false;

        if (tcp_connected) {
            // Constant on: TCP connected
            if (!led_state) {
                gpio_set_level(LED_GPIO, 1);
                led_state = true;
            }
            if (!logged_on || !prev_tcp_connected) {
                ESP_LOGI(TAG, "LED ON - TCP connected");
                logged_on = true;
            }
        } else if (wifi_connected) {
            // Fast blink: WiFi connected, TCP not
            if (now - last_blink_toggle >= fast_delay) {
                led_state = !led_state;
                gpio_set_level(LED_GPIO, led_state ? 1 : 0);
                last_blink_toggle = now;
                toggled = true;
            }
            if (!logged_fast || !prev_wifi_connected || prev_tcp_connected) {
                ESP_LOGI(TAG, "LED fast blink - WiFi connected, TCP not");
                logged_fast = true;
            }
        } else {
            // Slow blink: WiFi disconnected
            if (now - last_blink_toggle >= slow_delay) {
                led_state = !led_state;
                gpio_set_level(LED_GPIO, led_state ? 1 : 0);
                last_blink_toggle = now;
                toggled = true;
            }
            if (!logged_slow || prev_wifi_connected) {
                ESP_LOGI(TAG, "LED slow blink - WiFi disconnected");
                logged_slow = true;
            }
        }

        prev_wifi_connected = wifi_connected;
        prev_tcp_connected = tcp_connected;

        esp_task_wdt_reset();  // Feed WDT
        vTaskDelay(check_delay); // Periodic check
    }
    // vTaskDelete(NULL);  // Not reached; task runs forever
}