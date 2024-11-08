#include <stdio.h>
#include <stdlib.h>
#include "ble_main.h"
#include "ble_pwm.h"
#include "ble_gatts.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "string.h"

#define TXD_PIN 35
#define RXD_PIN 36

static void read_byte(void *arg)
{
    const uart_port_t uart_num = UART_NUM_1;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = UART_SCLK_DEFAULT,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, \
                                            uart_buffer_size, 10, &uart_queue, 0));


    char data[7];
    long converted = 80;
    while (1) {
        int len = uart_read_bytes(uart_num, data, 6, 10);
        // ESP_LOGI("UART TEST", "Recv str: %s | %d", (char *) data, len);
        if (len == 6) {
            data[len] = '\0';
            converted = strtol(data, NULL, 0);
            ESP_LOGI("UART TEST", "Flag: %d", flag);
            if(flag == false){
                setServoAngle(converted & 0xff);
                setHBridgePWM(converted >> 8, converted & 0xff);
                ESP_LOGI("UART TEST", "                         Servo data: %ld", converted & 0xff);
                ESP_LOGI("UART TEST", "                         H-bridge data: %ld", converted >> 8);
            }
        }
    }
}

void app_main(void)
{
    initPWM1();
    initPWM2and3();
    // initGPIO();
    bt_init();
    read_byte(NULL);
}