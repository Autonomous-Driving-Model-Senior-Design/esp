#include <driver/gpio.h>
#include "driver/ledc.h" 
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "ble.h"
#include "pwm.h"

BLEServer* pServer = NULL;
BLECharacteristic* ble = NULL;
BLECharacteristic* led = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
	deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
	deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic* led) {
		std::string value = led->getValue();
		if (value.length() > 0) {
			int receivedValue = static_cast<int>(value[0]);
			if (receivedValue == 1) {
				ledc_set_duty(LEDC_MODE, PWM_CHANNEL, 128); // 128/255 ~ 50% duty
				ledc_update_duty(LEDC_MODE, PWM_CHANNEL);
			} else if (receivedValue == 0) {
				ledc_set_duty(LEDC_MODE, PWM_CHANNEL, 0);
				ledc_update_duty(LEDC_MODE, PWM_CHANNEL);
			}
		}
	}
};

void initBLE() {
    // Create the BLE Device
	BLEDevice::init("ESP32");

	// Create the BLE Server
	pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());

	BLEService *pService = pServer->createService(SERVICE_UUID);

	// Create a BLE Characteristic
	ble = pService->createCharacteristic(
		SENSOR_CHARACTERISTIC_UUID,
		BLECharacteristic::PROPERTY_READ   |
		BLECharacteristic::PROPERTY_WRITE  |
		BLECharacteristic::PROPERTY_NOTIFY |
		BLECharacteristic::PROPERTY_INDICATE
	);

	// Create the ON button Characteristic
	led = pService->createCharacteristic(
		LED_CHARACTERISTIC_UUID,
		BLECharacteristic::PROPERTY_WRITE
	);

	led->setCallbacks(new MyCharacteristicCallbacks());

	// Create a BLE Descriptor
	ble->addDescriptor(new BLE2902());
	led->addDescriptor(new BLE2902());

	// Start the service
	pService->start();

	// Start advertising
	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->addServiceUUID(SERVICE_UUID);
	pAdvertising->setScanResponse(false);
	pAdvertising->setMinPreferred(0x0);
	BLEDevice::startAdvertising();
}