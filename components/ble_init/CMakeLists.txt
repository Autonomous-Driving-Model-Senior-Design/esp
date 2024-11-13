idf_component_register(SRCS "ble_main.c" "ble_gatts.c" "ble_adv.c" "ble_gap.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bt ble_pwm nvs_flash
                    PRIV_REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_uart)
