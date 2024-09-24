#include <stdio.h>
#include <stdlib.h>
#include "ble_main.h"
#include "ble_pwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "string.h"

#define TXD_PIN 1
#define RXD_PIN 2

static void echo_task(void *arg)
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

    // Write data to UART.
    // char* test_str = "This is a test string.";

    char data[1];

    while (1) {
        int len = uart_read_bytes(uart_num, data, 1, 1);
        // ESP_LOGI("UART TEST", "Recv str: %s", (char *) data);
        if (len) {
            data[len] = '\0';
            // ESP_LOGI("UART TEST", "Recv str: %s", (char *) data);
            ESP_LOGI("UART TEST", "Recv data: %d",strtol(data, NULL, 16));
            // setServoAngle(data);
        }
        

    }
}

void app_main(void)
{
    initPWM1();
    bt_init();
    echo_task(NULL);
}