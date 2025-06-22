#ifndef POWEROF_H
#define POWEROF_H

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Delay (in seconds) before entering system off after NFC field off */
#define SYSTEM_OFF_DELAY_S 5

    /* Initializes and schedules delayed system off work */
    void poweroff(void);

    /* Prints the reset reason to console */
    void print_reset_reason(void);

    /* Starts NFC emulation */
    int start_nfc(void);

#ifdef __cplusplus
}
#endif

#endif // POWEROF_H
