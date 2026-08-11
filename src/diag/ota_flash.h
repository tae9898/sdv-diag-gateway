/*
 * ota_flash.h - OTA flash API stub (gateway side).
 *
 * The gateway does not own flash (RAUC does). UDS OTA services 0x34/0x36/0x37
 * keep their handler shape but ota_flash_erase/write always fail, yielding
 * NRC 0x72 (general programming failure). Region macros preserved so the
 * request-validation path in uds_service.c is unchanged.
 *
 * (Shadows the obd-simulator header of the same name; that repo is read-only.)
 */
#ifndef OTA_FLASH_H
#define OTA_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* OTA data area -- last 8KB of flash (kept identical to the STM32 target). */
#define OTA_FLASH_BASE   0x0801E000U
#define OTA_FLASH_END    0x08020000U
#define OTA_FLASH_SIZE   (OTA_FLASH_END - OTA_FLASH_BASE)

/** @retval -1 always (gateway: RAUC owns flash; UDS OTA -> NRC 0x72). */
int ota_flash_erase(uint32_t addr, uint32_t size);

/** @retval -1 always (gateway: RAUC owns flash; UDS OTA -> NRC 0x72). */
int ota_flash_write(uint32_t addr, const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* OTA_FLASH_H */
