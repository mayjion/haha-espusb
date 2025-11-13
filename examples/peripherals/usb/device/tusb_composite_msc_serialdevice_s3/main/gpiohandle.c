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
#include "wificonfig.h"  // Added: For clear_nvs_config
#include "esp_wifi.h"    // Added: For esp_wifi_get_mac
#include "tcp_server.h"  // Added: For send_control_to_tcp and active_client
#include <stdio.h>       // Added: For snprintf
#include <string.h>      // Added: For strlen

static const char *TAG = "gpiohandle";
static EventGroupHandle_t led_event_group = NULL;  // For LED state signaling
static TaskHandle_t button_task_handle = NULL;  // For ISR notification
static bool factory_state = true;                // Added: Track factory mode (initial AP without saved creds)

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
                    // Modified: Check for client active and factory state before WiFi config send
                    bool client_active = false;
                    xSemaphoreTake(client_mutex, pdMS_TO_TICKS(100));
                    client_active = active_client.active;
                    xSemaphoreGive(client_mutex);
                    if (factory_state && client_active) {
                        uint8_t mac[6];
                        char new_ssid[32];
                        char new_password[64];
                        char mac_str[13];
                        esp_err_t mac_ret = esp_wifi_get_mac(WIFI_IF_AP, mac);
                        if (mac_ret != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to get MAC: %s", esp_err_to_name(mac_ret));
                        } else {
                            snprintf(mac_str, sizeof(mac_str), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                            snprintf(new_ssid, sizeof(new_ssid), "FUNLIGHT-%s", &mac_str[8]);
                            snprintf(new_password, sizeof(new_password), "funlight-%s", &mac_str[8]);
                            char message[128];
                            size_t msg_len = snprintf(message, sizeof(message), "WIFI=%s,%s\r\n", new_ssid, new_password);
                            esp_err_t send_ret = send_control_to_tcp(PROTO_TYPE_CMD, (const uint8_t *)message, msg_len);
                            if (send_ret == ESP_OK) {
                                ESP_LOGI(TAG, "Sent WiFi credentials to client: %s", message);
                                vTaskDelay(pdMS_TO_TICKS(2000));  // Ensure client receives notification
                                esp_err_t save_ret = save_wifi_credentials(new_ssid, new_password);  // Assumes function exists in wificonfig
                                if (save_ret == ESP_OK) {
                                    update_wifi_config(new_ssid, new_password);  // Assumes function exists in wificonfig
                                    factory_state = false;
                                    ESP_LOGI(TAG, "WiFi credentials updated, restarting");
                                    vTaskDelay(pdMS_TO_TICKS(1000));
                                    esp_restart();
                                } else {
                                    ESP_LOGE(TAG, "Failed to save WiFi credentials: %s", esp_err_to_name(save_ret));
                                }
                            } else {
                                ESP_LOGE(TAG, "Failed to send WiFi credentials to client: %s", esp_err_to_name(send_ret));
                            }
                        }
                    } else {
                        // Fallback: Original LED event if not factory or no client
                        xEventGroupSetBits(led_event_group, LED_EVENT_ACTIVE_BIT);
                        ESP_LOGI(TAG, "Button short click detected (%ums) - LED event set", ms_duration);
                    }
                } else if (ms_duration >= BUTTON_LONG_PRESS_MS) {  // >= for edge case
                    ESP_LOGW(TAG, "Button long press detected (%ums) - resetting", ms_duration);
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
    EventGroupHandle_t event_group = get_led_event_group();

    TickType_t last_blink_toggle = 0;
    bool led_state = false;
    const TickType_t fast_delay = pdMS_TO_TICKS(100);
    const TickType_t slow_delay = pdMS_TO_TICKS(500);

    while (1) {
        EventBits_t bits = xEventGroupWaitBits(event_group, LED_EVENT_ACTIVE_BIT, pdFALSE, pdFALSE, fast_delay);
        bool client_active = (bits & LED_EVENT_ACTIVE_BIT) || active_client.active;  // Fallback to shared var

        if (client_active) {
            gpio_set_level(LED_GPIO, 1);
            led_state = true;
        } else {
            // Blink mode: check frequently to respond to state changes
            TickType_t now = xTaskGetTickCount();
            if (now - last_blink_toggle >= slow_delay) {
                led_state = !led_state;
                gpio_set_level(LED_GPIO, led_state ? 1 : 0);
                last_blink_toggle = now;
            }
        }
        esp_task_wdt_reset();  // Feed WDT
    }
    // vTaskDelete(NULL);  // Not reached; task runs forever
}
