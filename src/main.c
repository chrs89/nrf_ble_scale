#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
/* this is for getting the SENSOR_CHAN_FORCE*/
#include "../drivers/sensor/nau7802/nau7802.h"
#include <shell_commands.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(main_logging);

// STEP 1 - Get the device nau7802 structure from the node label
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
static int sensor_data_ready_callback(const struct device *dev, const struct sensor_trigger *trig)
{
        int err;
        struct sensor_value force_val;

        /*TIME MEASUREMNT START*/
        // Get the current time (in milliseconds)
        uint32_t current_timestamp = k_uptime_get();

        // Calculate the time difference between the previous and current timestamp
        uint32_t time_diff = current_timestamp - prev_timestamp;

        /*TIME MEASUREMNT END*/

        // Log the time difference
        // LOG_INF("Time between callback calls: %d ms", time_diff);

        // Update the previous timestamp to the current one for the next callback
        prev_timestamp = current_timestamp;

        // LOG_INF("Sensor data is ready.");

        // STEP 9.3 - Continuously read out sensor data using the sensor API calls
        err = nau7802_sample_fetch(dev);
        if (err < 0)
        {
                LOG_ERR("Could not fetch sample (%d)", err);
                return -ENODATA;
        }

        // Get the force sensor value
        err = nau7802_channel_get(dev, SENSOR_CHAN_FORCE, &force_val);
        if (err < 0)
        {
                LOG_ERR("Could not get sample");
                return -ENODATA;
        }

        // LOG_INF("Force value: %d", force_val.val1);
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
                        return -ENODATA;;
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

int main(void)
{
        int err;
        int current_sample_rate_idx = 0; // Start with 10SPS
        struct sensor_value force_val;
        struct sensor_value raw_val;

        /* Init Shell Functions */
        SHELL_CMD_REGISTER(get_offset, NULL, "Fetch 50 sensor samples, average, and create calibration data", cmd_get_offset);
        SHELL_CMD_REGISTER(get_calFactor, NULL, "Fetch 50 sensor samples, average, and create calibration data", cmd_get_calFactor);
        SHELL_CMD_REGISTER(nau_suspend, NULL, "Suspend NAU7802 Thread", cmd_suspend_nau7802Thread);
        SHELL_CMD_REGISTER(nau_resume, NULL, "Resume NAU7802 Thread", cmd_resume_nau7802Thread);

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

        // Print log for main setup completion
        LOG_INF("MAIN: Setup complete. Waiting for Sensor Data...");

        // Main loop: Wait indefinitely for interrupts
        while (1)
        {
                // err = nau7802_sample_fetch(nau7802);
                // if (err < 0)
                // {
                //         LOG_ERR("Could not fetch sample (%d)", err);
                //         return;
                // }

                // // Get the force sensor value
                // err = nau7802_channel_get(nau7802, SENSOR_CHAN_FORCE, &force_val);
                // if (err < 0)
                // {
                //         LOG_ERR("Could not get sample");
                //         return;
                // }

                // // Get the force sensor value
                // err = nau7802_channel_get(nau7802, SENSOR_CHAN_RAW, &raw_val);
                // if (err < 0)
                // {
                //         LOG_ERR("Could not get sample");
                //         return;
                // }

                // LOG_INF("Force value: %f | Raw value: %d", sensor_value_to_float(&force_val), raw_val.val1);

                k_sleep(K_MSEC(1000)); // Delay between readings (adjust if needed)
        }

        return 0;
}
