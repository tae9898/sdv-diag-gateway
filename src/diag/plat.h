/*
 * plat.h - platform shim for the ported UDS stack.
 * Replaces STM32 HAL_GetTick() and Debug_Print() with Linux equivalents.
 */
#ifndef PLAT_H
#define PLAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Monotonic milliseconds since an arbitrary epoch (replaces HAL_GetTick). */
uint32_t platform_get_tick_ms(void);

/** printf-style log to stdout (replaces Debug_Print). */
void platform_log(const char *fmt, ...);

/**
 * Transmit one CAN(-FD) frame. len is the payload byte count (0..64); the
 * implementation handles FD DLC round-up. Returns 0 on success, -1 on error.
 * Implemented by the transport (socketcan_transport.cpp) or the test shim.
 */
int platform_can_send(uint32_t can_id, const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* PLAT_H */
