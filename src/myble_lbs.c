/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief LED Button Service (LBS) sample
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/sensor.h>

#include "myble_lbs.h"

LOG_MODULE_DECLARE(NAU7802, LOG_LEVEL_DBG);

// Global variable to store the period time (time between calls)
uint32_t prev_time = 0;

static bool notify_mysensor_enabled;
static bool indicate_enabled;
static bool button_state;
static struct my_lbs_cb lbs_cb;

/* STEP 4 - Define an indication parameter */
static struct bt_gatt_indicate_params ind_params;

/* STEP 3 - Implement the configuration change callback function */
static void mylbsbc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	indicate_enabled = (value == BT_GATT_CCC_INDICATE);
}

/* STEP 13 - Define the configuration change callback function for the MYSENSOR characteristic */
static void mylbsbc_ccc_mysensor_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify_mysensor_enabled = (value == BT_GATT_CCC_NOTIFY);
}

// This function is called when a remote device has acknowledged the indication at its host layer
static void indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err)
{
	LOG_DBG("Indication %s\n", err != 0U ? "fail" : "success");
}
static ssize_t write_led(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
						 uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (len != 1U)
	{
		LOG_DBG("Write led: Incorrect data length");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0)
	{
		LOG_DBG("Write led: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (lbs_cb.led_cb)
	{
		// Read the received value
		uint8_t val = *((uint8_t *)buf);

		if (val == 0x00 || val == 0x01)
		{
			// Call the application callback function to update the LED state
			lbs_cb.led_cb(val ? true : false);
		}
		else
		{
			LOG_DBG("Write led: Incorrect value");
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}
	}

	return len;
}

static ssize_t read_button(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
						   uint16_t len, uint16_t offset)
{
	// get a pointer to button_state which is passed in the BT_GATT_CHARACTERISTIC() and stored in attr->user_data
	const char *value = attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle, (void *)conn);

	if (lbs_cb.button_cb)
	{
		// Call the application callback function to update the get the current value of the button
		button_state = lbs_cb.button_cb();
		return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
	}

	return 0;
}

/* LED Button Service Declaration */
BT_GATT_SERVICE_DEFINE(
	my_lbs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_LBS),
	/* STEP 1 - Modify the Button characteristic declaration to support indication */
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_BUTTON, BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE,
						   BT_GATT_PERM_READ, read_button, NULL, &button_state),
	/* STEP 2 - Create and add the Client Characteristic Configuration Descriptor */
	BT_GATT_CCC(mylbsbc_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_LED, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL,
						   write_led, NULL),
	/* STEP 12 - Create and add the MYSENSOR characteristic and its CCCD  */
	BT_GATT_CHARACTERISTIC(BT_UUID_LBS_MYSENSOR, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
						   NULL, NULL),

	BT_GATT_CCC(mylbsbc_ccc_mysensor_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

);
/* A function to register application callbacks for the LED and Button characteristics  */
int my_lbs_init(struct my_lbs_cb *callbacks)
{
	if (callbacks)
	{
		lbs_cb.led_cb = callbacks->led_cb;
		lbs_cb.button_cb = callbacks->button_cb;
	}

	return 0;
}

/* STEP 5 - Define the function to send indications */
int my_lbs_send_button_state_indicate(bool button_state)
{
	if (!indicate_enabled)
	{
		return -EACCES;
	}
	ind_params.attr = &my_lbs_svc.attrs[2];
	ind_params.func = indicate_cb; // A remote device has ACKed at its host layer (ATT ACK)
	ind_params.destroy = NULL;
	ind_params.data = &button_state;
	ind_params.len = sizeof(button_state);
	return bt_gatt_indicate(NULL, &ind_params);
}

// Function to convert a 32-bit integer to little-endian format
static inline uint32_t to_little_endian(uint32_t val)
{
	return (val & 0xFF) << 24 | (val & 0xFF00) << 8 | (val & 0xFF0000) >> 8 | (val >> 24);
}

/* STEP 14 - Define the function to send notifications for the MYSENSOR characteristic */
int my_lbs_send_sensor_notify(struct sensor_value force_val)
{

	// Start the timer (milliseconds or ticks)
	uint32_t current_time = k_uptime_get_32(); // Get the current time (in ms)

	if (prev_time == 0)
	{
		// If this is the first call, initialize prev_time with current_time
		prev_time = current_time;
		LOG_INF("First call, no period time to calculate.");
		return 0; // Exit early on the first call, no period time to calculate
	}

	// Calculate the time difference between the current call and the previous call
	uint32_t period_time = current_time - prev_time;

	// Log the period time (time between successive calls)
	LOG_INF("Period time (time between calls): %d ms", period_time);

	// Now set the previous time to the current time for the next call
	prev_time = current_time;

	uint8_t buffer[8];
	int err;

	if (!notify_mysensor_enabled)
	{
		return -EACCES;
	}

	if (!USE_LITTLE_ENDIAN)
	{
		// Convert to little-endian format if system is big-endian
		uint32_t val1_le = to_little_endian(force_val.val1);
		uint32_t val2_le = to_little_endian(force_val.val2);
		memcpy(buffer, &val1_le, sizeof(uint32_t));		// Copy val1 (little-endian)
		memcpy(buffer + 4, &val2_le, sizeof(uint32_t)); // Copy val2 (little-endian)
	}
	else
	{
		// System is little-endian, directly copy values without conversion
		memcpy(buffer, &force_val.val1, sizeof(int32_t));
		memcpy(buffer + 4, &force_val.val2, sizeof(int32_t));

		// LOG_INF("Sensor val1 : % d val2 : % d, float_value: %f, buffer: %d", force_val.val1, force_val.val2, sensor_value_to_float(&force_val), buffer);
		// LOG_INF("buf[0] : %d, buf[1] : %d, buf[2] %d, buf[3] %d, buf[4] : %d, buf[5] : %d, buf[6] %d, buf[7] %d", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
	}

	err = bt_gatt_notify(NULL, &my_lbs_svc.attrs[7], &buffer, sizeof(buffer));
	if (err != 0)
	{
		LOG_ERR("bt_gatt_notify err: %d", err);
		return err;
	}

	return 0;
}
