#ifndef BLE_SERVICE_H_
#define BLE_SERVICE_H_

#include <zephyr/bluetooth/bluetooth.h>

int ble_service_init(void);
void ble_start_advertising(void);

#endif /* BLE_SERVICE_H_ */
