/*
 * diag_bridge.cpp - extern "C" glue between the SOME/IP service (C++) and the
 * ported UDS stack (C99). Mode A: the UDS engine runs in-process; the SOME/IP
 * payload carries the raw UDS request/response directly (no CAN/ISO-TP).
 */
#include "diag_bridge.h"

extern "C" {
#include "uds_service.h"
#include "diag_session.h"
#include "can_addressing.h"
}

/* Satisfies the `extern volatile uint8_t g_soft_reset_requested;` declared in
 * uds_service.h. Must have C linkage to match the C-side extern. */
extern "C" {
volatile uint8_t g_soft_reset_requested = 0;
}

extern "C" void diag_init(void)
{
    DiagSession_Init();
    UDS_Init();
    DiagSession_MarkBootReady();
}

extern "C" int diag_dispatch(const uint8_t *req, size_t req_len,
                             uint8_t *resp, size_t *resp_len)
{
    uint16_t rlen = 0U;
    /* In-process dispatch is always physical-addressed (1:1 SOME/IP request). */
    UDS_DispatchRequest(req, (uint16_t)req_len, ADDR_PHYSICAL, resp, &rlen);
    *resp_len = rlen;
    return (int)rlen;
}
