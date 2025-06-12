#ifndef SHELL_CMDS_H
#define SHELL_CMDS_H

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>



int cmd_hello_world(const struct shell *shell, size_t argc, char **argv);
int cmd_get_offset(const struct shell *shell, size_t argc, char **argv);
int cmd_get_calFactor(const struct shell *shell, size_t argc, char **argv);
int cmd_setCal_nvs(const struct shell *shell, size_t argc, char **argv);
void cmd_suspend_nau7802Thread(const struct shell *shell, size_t argc, char **argv);
void cmd_resume_nau7802Thread(const struct shell *shell, size_t argc, char **argv);
void cmd_nau_set_sps(const struct shell *shell, size_t argc, char **argv);

void register_shell_commands(void);


#endif // SHELL_COMMANDS_H
