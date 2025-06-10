#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include "shell_commands.h"


LOG_MODULE_REGISTER(shell_commands, LOG_LEVEL_INF);



/* ---- SHELL COMMAND IMPLEMENTATIONS ---- */

int cmd_hello_world(const struct shell *shell, size_t argc, char **argv)
{
    // Output "Hello, World!" in the shell
    shell_print(shell, "Hello, World!");
    return 0; // Return 0 to indicate success
}
