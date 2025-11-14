#ifndef GPIOHANDLE_H
#define GPIOHANDLE_H

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "tcp_client.h"  // For shared defines like LED_GPIO, BUTTON_GPIO, TAG, active_client

#define LED_GPIO          GPIO_NUM_7
#define BUTTON_GPIO       GPIO_NUM_0
#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_CLICK_THRESHOLD_MS 500
#define BUTTON_LONG_PRESS_MS 5000

// Event bits for LED state (from tcp_client or main)
#define LED_EVENT_ACTIVE_BIT BIT0

void button_init(void);
void button_task(void *pvParameters);
void led_init(void);
void led_task(void *pvParameters);
EventGroupHandle_t get_led_event_group(void);  // Getter for event group

#endif