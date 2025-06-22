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
#include "power_off/poweroff.h"

LOG_MODULE_REGISTER(MAIN_APPLICATION, LOG_LEVEL_DBG);

// === Definitions ===
#define RUN_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
#define USER_BUTTON DK_BTN1_MSK

#define STACKSIZE 1024
#define PRIORITY 2
#define RUN_LED_BLINK_INTERVAL 1000
#define BLE_CONN_INTERVAL_MS 40

typedef struct force_val_compr sens_val_t;
BUILD_ASSERT(sizeof(sens_val_t) == 4, "sens_val_t must be 4 bytes");

#define SAMPLE_SIZE 4            // 2 bytes val1 + 2 bytes val2
#define MAX_SAMPLES_PER_NOTIF 61 // 244 bytes max / 4 bytes

/*TASKS & THREAD DEFINITION*/
static void ble_send_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(ble_send_work, ble_send_work_handler);

void start_ble_send_loop(void)
{
    k_work_schedule(&ble_send_work, K_NO_WAIT);
}

K_MSGQ_DEFINE(sensor_data_queue, sizeof(sens_val_t), 128, 4);

K_SEM_DEFINE(bleSend_action_sem, 0, 1);

/*Thread Timing*/
#include <zephyr/sys/atomic.h>
static atomic_t producer_cb_counter = ATOMIC_INIT(0);
static atomic_t consumer_cb_counter = ATOMIC_INIT(0);
static atomic_t ble_loop_running = ATOMIC_INIT(0);
static struct k_thread log_rate_id;
K_THREAD_STACK_DEFINE(worker_stack_1, 512);

void log_callback_rate_thread(void)
{
    while (1)
    {
        k_sleep(K_SECONDS(1)); // Log every second

        int count = atomic_set(&producer_cb_counter, 0); // Reset and read
        int interval = 1e6 / count;
        printk("Producer Thread interval: %d µs, frequ: %d hz \n", interval, count);

        count = atomic_set(&consumer_cb_counter, 0); // Reset and read
        interval = 1e6 / count;
        printk("Consumer Thread interval: %d µs, frequ: %d hz \n", interval, count);
    }
}

/*Thread Timing*/

// === BLE Application Callbacks ===
static bool app_button_state;

static void app_led_cb(bool led_state)
{
    // dk_set_led(USER_LED, led_state);
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

    atomic_inc(&consumer_cb_counter);

    bool prod_active = k_msgq_num_used_get(&sensor_data_queue) > 0;

    uint8_t ble_buffer[244];
    struct force_val_compr val_compr;
    size_t sample_count = 0;

    while (sample_count < MAX_SAMPLES_PER_NOTIF)
    {
        if (k_msgq_get(&sensor_data_queue, &val_compr, K_NO_WAIT) == 0)
        {

            memcpy(&ble_buffer[sample_count * SAMPLE_SIZE], &val_compr, SAMPLE_SIZE);
            sample_count++;
        }
        else
        {
            break;
        }
    }

    if (sample_count > 0)
    {
        size_t len = sample_count * SAMPLE_SIZE;
        int err = my_lbs_send_batched_notify(ble_buffer, len);
        if (err)
        {
            LOG_ERR("Failed to send batched notify: %d", err);
        }
    }

    /*Adaptive Re-Scheduling if Producer is Active*/
    // Re-schedule for next interval (K_Work Delayable Concept)
    if (prod_active)
    {
        k_work_schedule(&ble_send_work, K_MSEC(BLE_CONN_INTERVAL_MS));
    }
    else
    {
        // Stop loop — it will be restarted by the producer when active again
        atomic_set(&ble_loop_running, 0);
        LOG_DBG("BLE loop stopped — no pending data");
    }
}

// === Sensor Callback ===
const struct device *const nau7802 = DEVICE_DT_GET_ONE(nuvoton_nau7802);

void sensor_data_ready_callback(const struct device *dev, const struct sensor_trigger *trig)
{
    /*TEST PURPOSE SIMULATE SENSOR DATA*/
    static uint16_t simulated_value = 0;
    static uint16_t val2 = 1234;

    atomic_inc(&producer_cb_counter); // Count every call

    struct sensor_value force_val;
    if (nau7802_sample_fetch(dev) < 0 || nau7802_channel_get(dev, SENSOR_CHAN_FORCE, &force_val) < 0)
    {
        LOG_ERR("Failed to fetch or get sensor value");
        return;
    }

    struct force_val_compr val_compr;
    val_compr.val1 = (int16_t)(simulated_value);
    val_compr.val2 = (int16_t)val2;

    // Increment and wrap
    simulated_value += 1;
    if (simulated_value > 10000)
    {
        simulated_value = 0;
    }

    if (k_msgq_put(&sensor_data_queue, &val_compr, K_NO_WAIT) == 0)
    {
        if (atomic_cas(&ble_loop_running, 0, 1))
        {
            // Start delayed BLE send work
            start_ble_send_loop();
            LOG_DBG("BLE loop started by producer");
        }
    }
    else
    {
        LOG_WRN("Sensor data queue full — dropping data");
    }

    // struct sensor_value force_val;
    // if (nau7802_sample_fetch(dev) < 0 || nau7802_channel_get(dev, SENSOR_CHAN_FORCE, &force_val) < 0)
    // {
    //     LOG_ERR("Failed to fetch or get sensor value");
    //     return;
    // }

    // struct force_val_compr val_compr;
    // val_compr.val1 = (int16_t)(force_val.val1);
    // val_compr.val2 = (int16_t)(force_val.val2 / 1000);

    // if (k_msgq_put(&sensor_data_queue, &val_compr, K_NO_WAIT) == 0)
    // {
    //     k_sem_give(&bleSend_action_sem); // Nothing to give
    // }
    // else
    // {
    //     LOG_WRN("Sensor data queue full — dropping data");
    // }
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
        LOG_ERR("Failed to load calib data from NVS, %d", err);
        // return 0;
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

    err = init_ble_service();
    if (err)
    {
        LOG_ERR("Failed to init BLE Service (err:%d)", err);
        return -1;
    }

    err = my_lbs_init(&app_callbacks);
    if (err)
    {
        LOG_ERR("Failed to init LBS (err:%d)", err);
        return -1;
    }

    // BLE advertising
    ble_start_advertising();

    // POWER_OFF
    poweroff();
    print_reset_reason();

    if (start_nfc() < 0)
    {
        printk("ERROR: NFC configuration failed\n");
        return -1;
    }

    // LOG TREAD ATOMMIC
    // k_thread_create(&log_rate_id, worker_stack_1, 512, (k_thread_entry_t)log_callback_rate_thread, NULL, NULL, NULL, 5, 0, K_NO_WAIT);

    // LED heartbeat loop
    for (;;)
    {
        dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
        k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
    }
}
