#include <driver/gpio.h>
#include <driver/uart.h>
#include "hal/uart_types.h"
#include "esp_err.h"

#define RX GPIO_NUM_44
#define TX GPIO_NUM_43

int readUART() {
    uint8_t data[1];
    int receivedValue = -1;

    int length = uart_read_bytes(UART_NUM_0, data, 1, 20 / portTICK_PERIOD_MS);

    if (length > 0) {
        receivedValue = data[0];
        if (receivedValue < 0 || receivedValue > 180) {
            receivedValue = -1;
        }
    }

    return receivedValue;
}

void initUART() {
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	// Configure UART parameters
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
	
	const int uart_buffer_size = (1024 * 2);
	QueueHandle_t uart_queue;
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, uart_buffer_size, uart_buffer_size, 0, NULL, 0));
	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void setup() {
	initUART();
}

void loop() {
	// Notify changed value
	//int data = readUART();
	const char data = 'A';  // Example data to send
	uart_write_bytes(UART_NUM_0, &data, 1);
	delay(1000);
}
