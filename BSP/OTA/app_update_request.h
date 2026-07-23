#ifndef APP_UPDATE_REQUEST_H
#define APP_UPDATE_REQUEST_H

#include <stdbool.h>

/* Replace this weak hook in application code to return the real update flag. */
bool App_IsUpdateRequested(void);

/* Compatibility API: requests Bootloader IAP for Slot B. */
void App_RequestBootloaderUpdate(void);

/* Request Bootloader IAP for Slot B. This is valid only while Slot A is confirmed. */
void App_RequestBootloaderUpdateSlotB(void);

/* Request Bootloader IAP for Slot A. This is valid only while Slot B is confirmed. */
void App_RequestBootloaderUpdateSlotA(void);

/* Switch to a valid Slot B image, or enter IAP for Slot B when it is unavailable. */
void App_RequestSwitchOrUpdateSlotB(void);

/* Switch to a valid Slot A image, or enter IAP for Slot A when it is unavailable. */
void App_RequestSwitchOrUpdateSlotA(void);

/* Request a verified Slot-A trial boot. Used by the Slot-B rollback command. */
void App_RequestBootSlotA(void);

#endif /* APP_UPDATE_REQUEST_H */
