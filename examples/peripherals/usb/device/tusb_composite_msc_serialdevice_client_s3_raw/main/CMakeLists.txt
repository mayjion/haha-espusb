idf_component_register(
    SRCS tcp_client.c gpiohandle.c usbhandle.c protocol.c wificonfig.c
    INCLUDE_DIRS .
    PRIV_REQUIRES fatfs wear_levelling esp_partition esp_wifi nvs_flash esp_driver_gpio esp_driver_uart esp_timer
)