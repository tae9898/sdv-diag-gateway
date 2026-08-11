/*
 * plat_linux.c - Linux platform shim: monotonic tick + stdout log.
 */
#include "plat.h"

#include <stdio.h>
#include <stdarg.h>
#include <time.h>

uint32_t platform_get_tick_ms(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL);
}

void platform_log(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    (void)vfprintf(stdout, fmt, ap);
    va_end(ap);
}
