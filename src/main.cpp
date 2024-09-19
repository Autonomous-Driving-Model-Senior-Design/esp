#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "driver/ledc.h" 
#include "hal/ledc_types.h" 
#include "hal/uart_types.h"
#include "esp_err.h"

BLEServer* server = NULL;
BLECharacteristic* ble = NULL;
BLECharacteristic* led = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

const gpio_num_t ledPin1 = GPIO_NUM_6;
const gpio_num_t ledPin2 = GPIO_NUM_7;
const gpio_num_t ledPin3 = GPIO_NUM_8;

#define RX GPIO_NUM_44
#define TX GPIO_NUM_43

#define PWM_CHANNEL1 LEDC_CHANNEL_1
#define PWM_CHANNEL2 LEDC_CHANNEL_2
#define PWM_CHANNEL3 LEDC_CHANNEL_3

#define PWM_FREQUENCY 50
#define HBRIDGE_PWM_FREQUENCY 100 

#define PWM_RESOLUTION LEDC_TIMER_8_BIT  // 0-255
#define LEDC_MODE LEDC_LOW_SPEED_MODE

#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500

#define SERVICE_UUID  "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define GPIO_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

class MyServerCallbacks: public BLEServerCallbacks {
	void onConnect(BLEServer* server) {
		deviceConnected = true;
	}

	void onDisconnect(BLEServer* server) {
		deviceConnected = false;
	}
};

void setServoAngle(int angle)
{
    if (angle < 0) {
		angle = 0;
	}
    else if (angle > 180) {
		angle = 180;
	}

    int pulseWidth = MIN_PULSE_WIDTH + ((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) * angle) / 180;

    uint32_t duty_max = (1 << PWM_RESOLUTION) - 1;
    uint32_t period_us = 1000000 / PWM_FREQUENCY;
    uint32_t duty = (pulseWidth * duty_max) / period_us;

    ledc_set_duty(LEDC_MODE, PWM_CHANNEL1, duty);
    ledc_update_duty(LEDC_MODE, PWM_CHANNEL1);
}

void setHBridgePWM(int dutyCycle) {
    if(dutyCycle == 190) { // Forward
     	ledc_set_duty(LEDC_MODE, PWM_CHANNEL2, 200);
     	ledc_set_duty(LEDC_MODE, PWM_CHANNEL3, 0);
    } else if(dutyCycle == 191) { // Backward
      	ledc_set_duty(LEDC_MODE, PWM_CHANNEL2, 0);
      	ledc_set_duty(LEDC_MODE, PWM_CHANNEL3, 200);
    } else if(dutyCycle == 192) { // Stop
      	ledc_set_duty(LEDC_MODE, PWM_CHANNEL2, 0);
      	ledc_set_duty(LEDC_MODE, PWM_CHANNEL3, 0);
    }
    ledc_update_duty(LEDC_MODE, PWM_CHANNEL2);
    ledc_update_duty(LEDC_MODE, PWM_CHANNEL3);
}

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic* led) {
		std::string value = led->getValue();
		if (value.length() > 0) {
			int receivedValue = static_cast<int>(value[0]);
			if(receivedValue <= 180) {
				setServoAngle(receivedValue); 
			}
			else {
				setHBridgePWM(receivedValue);
			}
		}
	}
};

// int readUART() {
//     uint8_t data[1];
//     int receivedValue = -1;

//     int length = uart_read_bytes(UART_NUM_0, data, 1, 20 / portTICK_PERIOD_MS);

//     if (length > 0) {
//         receivedValue = data[0];
//         if (receivedValue < 0 || receivedValue > 180) {
//             receivedValue = -1;
//         }
//     }

//     return receivedValue;
// }

// void initUART() {
// 	uart_config_t uart_config = {
// 		.baud_rate = 115200,
// 		.data_bits = UART_DATA_8_BITS,
// 		.parity = UART_PARITY_DISABLE,
// 		.stop_bits = UART_STOP_BITS_1,
// 		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
// 	};
// 	// Configure UART parameters
// 	ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
	
// 	const int uart_buffer_size = (1024 * 2);
// 	QueueHandle_t uart_queue;
// 	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, uart_buffer_size, uart_buffer_size, 10, 0, 0));
// 	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
// }

void initPWM1() {
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {
        .gpio_num = ledPin1,               
        .speed_mode = LEDC_MODE,
        .channel = PWM_CHANNEL1,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel);
    ledc_set_duty(LEDC_MODE, PWM_CHANNEL1, 0);
    ledc_update_duty(LEDC_MODE, PWM_CHANNEL1);
}

void initPWM2() {
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = HBRIDGE_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {
        .gpio_num = ledPin2,
        .speed_mode = LEDC_MODE,
        .channel = PWM_CHANNEL2,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel);
    ledc_set_duty(LEDC_MODE, PWM_CHANNEL2, 0);
    ledc_update_duty(LEDC_MODE, PWM_CHANNEL2);
}

void initPWM3() {
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_2,
        .freq_hz = HBRIDGE_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {
        .gpio_num = ledPin3,
        .speed_mode = LEDC_MODE,
        .channel = PWM_CHANNEL3,
        .timer_sel = LEDC_TIMER_2,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&pwm_channel);
    ledc_set_duty(LEDC_MODE, PWM_CHANNEL3, 0);
    ledc_update_duty(LEDC_MODE, PWM_CHANNEL3);
}

void setup() {
	Serial.begin(115200);
	initPWM1();
	initPWM2();
	initPWM3();
	// initUART();

	BLEDevice::init("ESP32");

	// Create the BLE Server
	server = BLEDevice::createServer();
	server->setCallbacks(new MyServerCallbacks());
	BLEService *service = server->createService(SERVICE_UUID);
	ble = service->createCharacteristic(
		SENSOR_CHARACTERISTIC_UUID,
		BLECharacteristic::PROPERTY_READ |
		BLECharacteristic::PROPERTY_WRITE |
		BLECharacteristic::PROPERTY_NOTIFY |
		BLECharacteristic::PROPERTY_INDICATE
	);

	// Create the ON button Characteristic
	led = service->createCharacteristic(
		GPIO_CHARACTERISTIC_UUID,
		BLECharacteristic::PROPERTY_WRITE
	);

	// Register the callback for the ON button characteristic
	led->setCallbacks(new MyCharacteristicCallbacks());

	// Create a BLE Descriptor
	ble->addDescriptor(new BLE2902());
	led->addDescriptor(new BLE2902());
	service->start();

	// Start advertising
	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(SERVICE_UUID);
	pAdvertising->setScanResponse(false);
	pAdvertising->setMinPreferred(0x0);
	BLEDevice::startAdvertising();
}

void loop() {
	// Notify changed value
	// int data = readUART();
	// Serial.println(data);
	if (deviceConnected) {
		ble->setValue(String(value).c_str());
		ble->notify();
		value++;
		delay(3000);
	}

	// Disconnecting
	if (!deviceConnected && oldDeviceConnected) {
		delay(500);
		server->startAdvertising();
		oldDeviceConnected = deviceConnected;
	}

	// Connecting
	if (deviceConnected && !oldDeviceConnected) {
		oldDeviceConnected = deviceConnected;
	}
}

// #include <driver/gpio.h>
// #include <driver/uart.h>
// #include "hal/uart_types.h"
// #include "esp_err.h"
// #include <Arduino.h>

// int readSingleByteInteger() {
//     uint8_t data[1];
//     int receivedValue = -1;

//     int length = uart_read_bytes(UART_NUM_1, data, sizeof(data), 20 / portTICK_PERIOD_MS);

//     if (length > 0) {
//         receivedValue = data[0]; // Directly use the byte value
//     }

//     return receivedValue;
// }


// void initUART1() {
// 	uart_config_t uart_config = {
// 		.baud_rate = 115200,
// 		.data_bits = UART_DATA_8_BITS,
// 		.parity = UART_PARITY_DISABLE,
// 		.stop_bits = UART_STOP_BITS_1,
// 		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
// 	};
// 	// Configure UART parameters
// 	ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
	
// 	const int uart_buffer_size = (1024 * 2);
// 	QueueHandle_t uart_queue;
// 	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, SOC_TX0, SOC_RX0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

// 	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, uart_buffer_size, uart_buffer_size, 0, NULL, 0));
// }

// void initUART2() {
// 	uart_config_t uart_config = {
// 		.baud_rate = 115200,
// 		.data_bits = UART_DATA_8_BITS,
// 		.parity = UART_PARITY_DISABLE,
// 		.stop_bits = UART_STOP_BITS_1,
// 		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
// 	};
// 	// Configure UART parameters
// 	ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
	
// 	const int uart_buffer_size = (1024 * 2);
// 	QueueHandle_t uart_queue;
// 	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TX1, RX1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

// 	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, uart_buffer_size, uart_buffer_size, 0, NULL, 0));
// }

// void setup() {
// 	Serial.begin(115200);
// 	initUART1();
// 	initUART2();
// }

// void loop() {
// 	// Notify changed value
// 	//int data = readUART();
// 	int data = readSingleByteInteger();
// 	Serial.println(data);
// 	uart_write_bytes(UART_NUM_0, &data, 1);
// 	// delay(1000);
// }