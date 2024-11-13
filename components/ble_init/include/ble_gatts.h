#pragma once

#include "esp_gatts_api.h"

extern bool flag;
extern bool uart_running;
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);