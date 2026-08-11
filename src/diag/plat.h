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

#ifdef __cplusplus
}
#endif

#endif /* PLAT_H */
