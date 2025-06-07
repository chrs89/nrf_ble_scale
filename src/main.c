#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
/* this is for getting the SENSOR_CHAN_FORCE*/
#include "../drivers/sensor/nau7802/nau7802.h"
/*Bluetooth*/
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <dk_buttons_and_leds.h>
#include <myble_lbs.h>

/*Shell Specific - Developement*/
#include <shell_commands.h>
#include <zephyr/shell/shell.h>

/*Thread Analyzer*/
#include <zephyr/kernel/thread.h>

LOG_MODULE_REGISTER(MAIN_APPLICATION, LOG_LEVEL_DBG);

static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
    (BT_LE_ADV_OPT_CONN |
     BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use identity address */
    800,                          /* Min Advertising Interval 500ms (800*0.625ms) */
    801,                          /* Max Advertising Interval 500.625ms (801*0.625ms) */
    NULL);                        /* Set to NULL for undirected advertising */

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define CON_STATUS_LED DK_LED2
#define USER_LED DK_LED3
#define USER_BUTTON DK_BTN1_MSK

#define STACKSIZE 1024
#define PRIORITY 7

#define QUEUE_SIZE 10

// Declare the message queue
typedef struct sensor_value sens_val_t;
K_MSGQ_DEFINE(sensor_data_queue, sizeof(sens_val_t), QUEUE_SIZE, 4);
K_SEM_DEFINE(bleSend_action_sem, 0, 1); // Semaphore to signal the worker
#define RUN_LED_BLINK_INTERVAL 1000
/* STEP 17 - Define the interval at which you want to send data at */
#define NOTIFY_INTERVAL 500
static bool app_button_state;
static struct k_work adv_work;
static struct k_work ble_send_work;
// Declare a worker thread and its stack
#define STACKSIZE 1024
#define PRIORITY 7
K_THREAD_STACK_DEFINE(worker_stack, STACKSIZE);
static struct k_thread worker_thread_data;

static uint32_t app_sensor_value = 100;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),

};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LBS_VAL),
};

static void adv_work_handler(struct k_work *work)
{
        int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

        if (err)
        {
                printk("Advertising failed to start (err %d)\n", err);
                return;
        }

        printk("Advertising successfully started\n");
}

static void advertising_start(void)
{
        k_work_submit(&adv_work);
}
static void recycled_cb(void)
{
        printk("Connection object available from previous conn. Disconnect is complete!\n");
        advertising_start();
}

/* Work handler to process Send BLE Data */
static void ble_send_work_handler(struct k_work *work)
{
        /* Process everything in the message queue */
        LOG_DBG("Enter: ble_send_work_handler");
        int err;
        while (1)
        {
                struct sensor_value force_val;
                err = k_msgq_get(&sensor_data_queue, &force_val, K_FOREVER); // Block if the queue is empty

                if (err == 0)
                {
                        // If the system is little-endian, no conversion is needed
                        // Otherwise, convert to little-endian before sending

                        // Send the packed buffer over BLE
                        my_lbs_send_sensor_notify(force_val);
                }

                // Yield the thread to avoid blocking other work
                k_yield();
        }
        LOG_DBG("Enter: ble_send_work_handler");
}

static void simulate_data(void)
{
        app_sensor_value++;
        if (app_sensor_value == 200)
        {
                app_sensor_value = 100;
        }
}
static void app_led_cb(bool led_state)
{
        dk_set_led(USER_LED, led_state);
}

static bool app_button_cb(void)
{
        return app_button_state;
}

static struct my_lbs_cb app_callbacks = {
    .led_cb = app_led_cb,
    .button_cb = app_button_cb,
};

static void button_changed(uint32_t button_state, uint32_t has_changed)
{
        if (has_changed & USER_BUTTON)
        {
                uint32_t user_button_state = button_state & USER_BUTTON;
                /* STEP 6 - Send indication on a button press */
                my_lbs_send_button_state_indicate(user_button_state);
                app_button_state = user_button_state ? true : false;
        }
}
static void on_connected(struct bt_conn *conn, uint8_t err)
{
        if (err)
        {
                printk("Connection failed (err %u)\n", err);
                return;
        }

        printk("Connected\n");

        dk_set_led_on(CON_STATUS_LED);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
        printk("Disconnected (reason %u)\n", reason);

        dk_set_led_off(CON_STATUS_LED);
}

struct bt_conn_cb connection_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
    .recycled = recycled_cb,
};

static int init_button(void)
{
        int err;

        err = dk_buttons_init(button_changed);
        if (err)
        {
                printk("Cannot init buttons (err: %d)\n", err);
        }

        return err;
}

/*Init Sensor Devices*/
const struct device *const nau7802 = DEVICE_DT_GET_ONE(nuvoton_nau7802);

// Store the previous timestamp (in milliseconds)
static uint32_t prev_timestamp = 0;

// Function to set the sample rate
static int main_set_sample_rate(const struct device *dev, uint16_t rate_idx)
{
        const struct nau7802_config *config = dev->config;
        int ret;

        // Change the conversion rate by using the index to get the correct sample rate
        ret = nau7802_setRate_runtime(dev, config, sampleRateMap[rate_idx]);
        if (ret != 0)
        {
                LOG_ERR("Failed to set the sample rate");
                return ret;
        }

        LOG_INF("Sample rate set to %d SPS", sampleRateMap[rate_idx]);
        return 0;
}

// Callback function to handle the interrupt when new data is ready
void sensor_data_ready_callback(const struct device *dev, const struct sensor_trigger *trig)
{
        int err;
        struct sensor_value force_val;

        /*On Callback DRDY*/
        err = nau7802_sample_fetch(dev);
        if (err < 0)
        {
                LOG_ERR("Could not fetch sample (%d)", err);
        }

        // Get the force sensor value
        err = nau7802_channel_get(dev, SENSOR_CHAN_FORCE, &force_val);
        if (err < 0)
        {
                LOG_ERR("Could not get sample, (%d)", err);
        }
        // Send the acquired data to the queue
        k_msgq_put(&sensor_data_queue, &force_val, K_NO_WAIT);
        k_sem_give(&bleSend_action_sem);
        //  k_sleep(K_MSEC(100));
}

void bleSend_action_worker(void)
{
        while (1)
        {
                // Wait for the signal from the interrupt handler
                k_sem_take(&bleSend_action_sem, K_FOREVER);

                // Now submit the work in a non-interrupt context (safe to do here)
                k_work_submit(&ble_send_work);
        }
}

// Function to fetch 10 sensor samples, average them, and create the calibration data
int get_offset_data(const struct device *dev)
{
        int err;
        int i;
        int sensor_value = 0;
        int32_t sum = 0;
        int ret;
        int iter = 100;
        struct sensor_value force_val;

        struct nau7802_data *data = nau7802->data;

        // Fetch 10 sensor samples and calculate the sum
        for (i = 0; i < iter; i++)
        {
                err = nau7802_sample_fetch(dev);
                if (err < 0)
                {
                        LOG_ERR("Could not fetch sample (%d)", err);
                        return -ENODATA;
                }

                // Get the force sensor value
                err = nau7802_channel_get(dev, SENSOR_CHAN_RAW, &force_val);
                if (err < 0)
                {
                        LOG_ERR("Could not get sample");
                        return -ENODATA;
                        ;
                }

                LOG_INF("Force value: %d", force_val.val1);

                if (i > 49)
                {
                        sum += force_val.val1; // Accumulate sensor values
                }
                k_sleep(K_MSEC(10)); // Delay between readings (adjust if needed)
        }

        // Calculate the average
        data->zero_offset = -(sum / 50); // Average the offset (sensor value)
        LOG_INF("Calculated offset: %d", data->zero_offset);

        return 0;
}

// Shell command function to execute the fetch_and_create_calibration_data function
int cmd_get_offset(const struct shell *shell, size_t argc, char **argv)
{

        int err;

        err = get_offset_data(nau7802);
        if (err != 0)
        {
                LOG_ERR("Error func:fetch_and_create_calibration_data, %d", err);
        }

        return 0;
}

int get_calFactor_data(const struct device *dev, float32_t cal_weight)
{
        int err;
        int i;
        int sensor_value = 0;
        float32_t sum = 0;
        int ret;
        int iter = 100;
        struct sensor_value force_val;

        struct nau7802_data *data = nau7802->data;

        // Fetch 10 sensor samples and calculate the sum
        for (i = 0; i < iter; i++)
        {
                err = nau7802_sample_fetch(dev);
                if (err < 0)
                {
                        LOG_ERR("Could not fetch sample (%d)", err);
                        return -ENODATA;
                }

                // Get the force sensor value
                err = nau7802_channel_get(dev, SENSOR_CHAN_RAW, &force_val);
                if (err < 0)
                {
                        LOG_ERR("Could not get sample");
                        return -ENODATA;
                        ;
                }

                LOG_INF("Force value: %d", force_val.val1);

                if (i > 49)
                {
                        sum += force_val.val1; // Accumulate sensor values
                }
                k_sleep(K_MSEC(10)); // Delay between readings (adjust if needed)
        }

        // Calculate the average
        data->calibration_factor = ((sum / 50) - data->zero_offset) / cal_weight;
        LOG_INF("Calculated Calibration Factor: %f | Calibration Weight: %f", data->calibration_factor, cal_weight);

        return 0;
}

// Shell command function to execute the fetch_and_create_calibration_data function
int cmd_get_calFactor(const struct shell *shell, size_t argc, char **argv)
{

        if (argc != 2) // Ensure one argument is passed
        {
                shell_print(shell, "Error: Please provide the calibration weight.");
                return -EINVAL; // Return error if arguments are incorrect
        }

        // Convert the argument to float32_t (cal_weight)
        float32_t cal_weight = strtof(argv[1], NULL); // Convert the argument (argv[1]) to a float

        if (cal_weight == 0)
        {
                shell_print(shell, "Error: Invalid calibration weight.");
                return -EINVAL; // Return error if conversion fails
        }

        int err;

        err = get_calFactor_data(nau7802, cal_weight);
        if (err != 0)
        {
                LOG_ERR("Error func:fetch_and_create_calibration_data, %d", err);
                return err;
        }

        return 0;
}

int set_calData_nvs(const struct device *dev)
{
        int err;

        struct nau7802_data *data = nau7802->data;

        err = store_calibration_data_nvs(data->zero_offset, (float32_t)data->calibration_factor);
        if (err != 0)
        {
                LOG_ERR("Error func:set_calData_nvs, %d", err);
                return err;
        }

        return 0;
}

int cmd_setCal_nvs(const struct shell *shell, size_t argc, char **argv)
{

        int err;

        err = set_calData_nvs(nau7802);
        if (err != 0)
        {
                LOG_ERR("Error func:fetch_and_create_calibration_data, %d", err);
        }

        return 0;
}

void cmd_suspend_nau7802Thread(const struct shell *shell, size_t argc, char **argv)
{
        struct nau7802_data *data = nau7802->data;
        int err;

        nau7802_ownThreadSuspend(nau7802);
}

void cmd_resume_nau7802Thread(const struct shell *shell, size_t argc, char **argv)
{
        struct nau7802_data *data = nau7802->data;
        int err;

        nau7802_ownThreadResume(nau7802);
}

void cmd_nau_set_sps(const struct shell *shell, size_t argc, char **argv)
{
        int err;
        int user_input;
        struct sensor_value sps; // Declare a local sensor_value structure

        // Check if the argument count is correct (i.e., there is exactly one user input)
        if (argc != 2)
        {
                shell_print(shell, "Error: Please provide a valid sample rate (0...4).");
                return;
        }

        // Convert the user input argument to an integer
        user_input = atoi(argv[1]); // Convert string input to integer

        // Validate user input is within the allowed range
        if (user_input < 0 || user_input > 4)
        {
                LOG_DBG("Error: Invalid input. Please enter a number between 0 and 4.");
                return;
        }

        // Set the corresponding sample rate in the sps structure
        sps.val1 = sampleRateMap[user_input]; // Assign mapped sample rate to sps.val1

        // Call nau7802_attr_set to set the sample rate
        err = nau7802_attr_set(nau7802, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &sps);
        if (err)
        {
                LOG_DBG("Error: Failed to set sampling rate: %d", err);
        }
        else
        {
                LOG_DBG("Sampling rate set to: %d", sps.val1); // Log the set sample rate
        }
}

int main(void)
{
        int err;

        /* Init Shell Functions */
        SHELL_CMD_REGISTER(get_offset, NULL, "Fetch 50 sensor samples, average, and create calibration data", cmd_get_offset);
        SHELL_CMD_REGISTER(get_calFactor, NULL, "Fetch 50 sensor samples, average, and create calibration data", cmd_get_calFactor);
        SHELL_CMD_REGISTER(nau_suspend, NULL, "Suspend NAU7802 Thread", cmd_suspend_nau7802Thread);
        SHELL_CMD_REGISTER(nau_resume, NULL, "Resume NAU7802 Thread", cmd_resume_nau7802Thread);
        SHELL_CMD_REGISTER(nau_set_rate, NULL, "nau set 320sps", cmd_nau_set_sps);

        // Ensure the device is ready before proceeding
        if (!device_is_ready(nau7802))
        {
                LOG_ERR("Sensor device not ready.");
                return 0;
        }

        // Register the callback for the Data Ready trigger
        struct sensor_trigger trig = {
            .type = SENSOR_TRIG_DATA_READY,
            .chan = SENSOR_CHAN_FORCE,
        };

        // Register the callback function for the trigger
        err = nau7802_trigger_set(nau7802, &trig, sensor_data_ready_callback);
        if (err < 0)
        {
                LOG_ERR("Could not set trigger callback (%d)", err);
                return 0;
        }

        /*Init BLE & Services*/
        int blink_status = 0;

        err = dk_leds_init();
        if (err)
        {
                LOG_ERR("LEDs init failed (err %d)\n", err);
                return -1;
        }

        err = init_button();
        if (err)
        {
                printk("Button init failed (err %d)\n", err);
                return -1;
        }

        err = bt_enable(NULL);
        if (err)
        {
                LOG_ERR("Bluetooth init failed (err %d)\n", err);
                return -1;
        }
        bt_conn_cb_register(&connection_callbacks);

        err = my_lbs_init(&app_callbacks);
        if (err)
        {
                printk("Failed to init LBS (err:%d)\n", err);
                return -1;
        }
        LOG_INF("Bluetooth initialized\n");
        k_work_init(&ble_send_work, ble_send_work_handler);
        k_work_init(&adv_work, adv_work_handler);
        advertising_start();

        // Start the worker thread for processing button actions
        k_thread_create(&worker_thread_data, worker_stack, STACKSIZE,
                        (k_thread_entry_t)ble_send_work_handler, NULL, NULL, NULL,
                        PRIORITY, 0, K_NO_WAIT);
        k_thread_name_set(&worker_thread_data, "BLE Send Thread");
        for (;;)
        {
                dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
                k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
                // thread_analyzer_print();
        }
}
