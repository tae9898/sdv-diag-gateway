/*
 * ota_stub.c - OTA flash stub implementations (gateway uses RAUC, not UDS flash).
 * Returning -1 makes UDS 0x34/0x36/0x37 respond NRC 0x72 (general programming failure).
 */
#include "ota_flash.h"

int ota_flash_erase(uint32_t addr, uint32_t size)
{
    (void)addr;
    (void)size;
    return -1;
}

int ota_flash_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    (void)addr;
    (void)data;
    (void)len;
    return -1;
}
