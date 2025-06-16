#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel/thread.h>
#include <dk_buttons_and_leds.h>

#include "sensor/nau7802/nau7802.h"

#ifdef CONFIG_APP_ENABLE_SHELL_CMDS
#include "shell/shell_commands.h"
#endif

#include "ble/ble_service.h"
#include "ble/myble_lbs.h"
#include "sensor/appcall_nau7802.h"
#include "app_nau7802_cmd.h"

LOG_MODULE_REGISTER(MAIN_APPLICATION, LOG_LEVEL_DBG);

// === Definitions ===
#define RUN_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
#define USER_LED DK_LED3
#define USER_BUTTON DK_BTN1_MSK

#define STACKSIZE 1024
#define PRIORITY 7
#define QUEUE_SIZE 10
#define RUN_LED_BLINK_INTERVAL 1000

typedef struct sensor_value sens_val_t;

K_MSGQ_DEFINE(sensor_data_queue, sizeof(sens_val_t), QUEUE_SIZE, 4);
K_SEM_DEFINE(bleSend_action_sem, 0, 1);

K_THREAD_STACK_DEFINE(worker_stack, STACKSIZE);
static struct k_thread worker_thread_data;

static struct k_work ble_send_work;

static bool app_button_state;

// === BLE Application Callbacks ===
static void app_led_cb(bool led_state)
{
    dk_set_led(USER_LED, led_state);
}

static bool app_button_cb(void)
{
    return app_button_state;
}

static void app_aindx_cb(const uint8_t val)
{
    LOG_INF("A_INDX_Call_Back_Function value: %d", val);
    if (val < NAU7802_CMD_COUNT)
    {
        // Value is a valid command enum
        int err = nau7802_execute_command((enum app_nau7802_command)val);
        if (err)
        {
            LOG_ERR("nau7802_execute_command (err: %d)", err);
        }
    }
    else
    {
        LOG_WRN("Invalid BLE command: %d", val);
    }
}

static struct my_lbs_cb app_callbacks = {
    .led_cb = app_led_cb,
    .button_cb = app_button_cb,
    .aindx_cb = app_aindx_cb,
};

// === Button Callback ===
static void button_changed(uint32_t button_state, uint32_t has_changed)
{
    if (has_changed & USER_BUTTON)
    {
        uint32_t user_button_state = button_state & USER_BUTTON;
        my_lbs_send_button_state_indicate(user_button_state);
        app_button_state = user_button_state ? true : false;
    }
}

// === BLE Send Work Handler ===
static void ble_send_work_handler(struct k_work *work)
{
    LOG_DBG("Enter: ble_send_work_handler");
    while (1)
    {
        struct sensor_value force_val;
        if (k_msgq_get(&sensor_data_queue, &force_val, K_FOREVER) == 0)
        {
            my_lbs_send_sensor_notify(force_val);
        }
        k_yield();
    }
}

// === BLE Worker Thread ===
void bleSend_action_worker(void)
{
    while (1)
    {
        k_sem_take(&bleSend_action_sem, K_FOREVER);
        k_work_submit(&ble_send_work);
    }
}

// === Sensor Callback ===
const struct device *const nau7802 = DEVICE_DT_GET_ONE(nuvoton_nau7802);

void sensor_data_ready_callback(const struct device *dev, const struct sensor_trigger *trig)
{
    struct sensor_value force_val;
    if (nau7802_sample_fetch(dev) < 0 || nau7802_channel_get(dev, SENSOR_CHAN_FORCE, &force_val) < 0)
    {
        LOG_ERR("Failed to fetch or get sensor value");
        return;
    }

    k_msgq_put(&sensor_data_queue, &force_val, K_NO_WAIT);
    k_sem_give(&bleSend_action_sem);
}

// === Init Button ===
static int init_button(void)
{
    int err = dk_buttons_init(button_changed);
    if (err)
    {
        LOG_ERR("Cannot init buttons (err: %d)", err);
    }
    return err;
}

// === Main Entry Point ===
int main(void)
{
    int err;
    int blink_status = 0;

// Shell commands
#ifdef CONFIG_APP_ENABLE_SHELL_CMDS
    register_shell_commands();
#endif

    // Sensor readiness
    if (!device_is_ready(nau7802))
    {
        LOG_ERR("Sensor not ready");
        return 0;
    }

// Load Calibration Data from NVS and set to nau7802
#ifdef CONFIG_APP_ENABLE_NVSRW
    err = load_calib_fromNVS(nau7802);
    if (err < 0)
    {
        LOG_ERR("Failed to set load calib data from NVS, %d", err);
        return 0;
    }
#endif

    // Set up sensor trigger
    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_FORCE,
    };

    err = nau7802_trigger_set(nau7802, &trig, sensor_data_ready_callback);
    if (err < 0)
    {
        LOG_ERR("Failed to set sensor trigger");
        return 0;
    }

    // LED & button init
    err = dk_leds_init();
    if (err)
        return -1;

    err = init_button();
    if (err)
        return -1;

    // BLE service
    err = ble_service_init();
    if (err)
        return -1;

    err = my_lbs_init(&app_callbacks);
    if (err)
    {
        LOG_ERR("Failed to init LBS (err:%d)", err);
        return -1;
    }

    // BLE advertising + work
    k_work_init(&ble_send_work, ble_send_work_handler);
    ble_start_advertising();

    // BLE Send Worker Thread
    k_thread_create(&worker_thread_data, worker_stack, STACKSIZE,
                    (k_thread_entry_t)ble_send_work_handler,
                    NULL, NULL, NULL,
                    PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&worker_thread_data, "BLE Send Thread");

    // LED heartbeat loop
    for (;;)
    {
        dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
        k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
    }
}
