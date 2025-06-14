#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "shell_commands.h"
#include "sensor/nau7802/nau7802.h"
#include "../sensor/calibration.h"

LOG_MODULE_REGISTER(shell_commands, LOG_LEVEL_DBG);

extern const struct device *const nau7802;

/* ---- SHELL COMMAND IMPLEMENTATIONS ---- */

int cmd_hello_world(const struct shell *shell, size_t argc, char **argv)
{
    // Output "Hello, World!" in the shell
    shell_print(shell, "Hello, World!");
    return 0; // Return 0 to indicate success
}

int cmd_get_offset(const struct shell *shell, size_t argc, char **argv)
{
    return get_offset_data(nau7802);
}

int cmd_get_calFactor(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 2)
    {
        shell_print(shell, "Error: Provide calibration weight.");
        return -EINVAL;
    }

    float32_t cal_weight = strtof(argv[1], NULL);
    if (cal_weight == 0)
    {
        shell_print(shell, "Error: Invalid calibration weight.");
        return -EINVAL;
    }

    return get_calFactor_data(nau7802, cal_weight);
}

int cmd_setCal_nvs(const struct shell *shell, size_t argc, char **argv)
{
    return set_calData_nvs(nau7802);
}

void cmd_suspend_nau7802Thread(const struct shell *shell, size_t argc, char **argv)
{
    nau7802_ownThreadSuspend(nau7802);
}

void cmd_resume_nau7802Thread(const struct shell *shell, size_t argc, char **argv)
{
    nau7802_ownThreadResume(nau7802);
}

void cmd_nau_set_sps(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 2)
    {
        shell_print(shell, "Error: Provide sample rate (0–4).");
        return;
    }

    int user_input = atoi(argv[1]);
    if (user_input < 0 || user_input > 4)
    {
        shell_print(shell, "Invalid input. Enter number 0–4.");
        return;
    }

    struct sensor_value sps = {.val1 = sampleRateMap[user_input]};
    int err = nau7802_attr_set(nau7802, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &sps);
    if (err)
    {
        LOG_DBG("Failed to set sampling rate: %d", err);
    }
    else
    {
        LOG_DBG("Sampling rate set to: %d", sps.val1);
    }
}

void register_shell_commands(void)
{
    SHELL_CMD_REGISTER(get_offset, NULL, "Get offset", cmd_get_offset);
    SHELL_CMD_REGISTER(get_calFactor, NULL, "Get calibration factor", cmd_get_calFactor);
    SHELL_CMD_REGISTER(set_calNVS, NULL, "Set calibration to NVS", cmd_setCal_nvs);
    SHELL_CMD_REGISTER(nau_suspend, NULL, "Suspend NAU7802", cmd_suspend_nau7802Thread);
    SHELL_CMD_REGISTER(nau_resume, NULL, "Resume NAU7802", cmd_resume_nau7802Thread);
    SHELL_CMD_REGISTER(nau_set_rate, NULL, "Set NAU7802 rate", cmd_nau_set_sps);
    SHELL_CMD_REGISTER(hello, NULL, "Say Hello", cmd_hello_world);
}
