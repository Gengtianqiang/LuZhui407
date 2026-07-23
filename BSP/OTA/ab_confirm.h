#ifndef AB_CONFIRM_H
#define AB_CONFIRM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_BOOT_CONFIRM_OK = 0U,
    APP_BOOT_CONFIRM_BAD_MAGIC,
    APP_BOOT_CONFIRM_BAD_COMMIT,
    APP_BOOT_CONFIRM_BAD_SLOT_FIELD,
    APP_BOOT_CONFIRM_BAD_CRC,
    APP_BOOT_CONFIRM_PENDING_OTHER_SLOT,
    APP_BOOT_CONFIRM_NO_SPACE,
    APP_BOOT_CONFIRM_WRITE_FAILED
} app_boot_confirm_status_t;

/* Confirm this Slot-A application after its critical initialization succeeds. */
app_boot_confirm_status_t app_boot_confirm(void);

#endif /* AB_CONFIRM_H */
