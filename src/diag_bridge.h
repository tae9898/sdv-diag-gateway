/*
 * diag_bridge.h - C++ facing declarations for the in-process UDS engine (Mode A).
 *
 * The SOME/IP service includes only this header; the extern "C" UDS stack lives
 * behind diag_bridge.cpp. The payload of a SOME/IP request IS the raw UDS request
 * byte stream; the engine returns the raw UDS response byte stream.
 */
#ifndef DIAG_BRIDGE_H
#define DIAG_BRIDGE_H

#include <cstddef>
#include <cstdint>

/* Response buffer size; matches UDS_MAX_RESPONSE_SIZE in the C stack. */
#define DIAG_RESP_MAX 64u

#ifdef __cplusplus
extern "C" {
#endif

/** Initialise the session manager, UDS dispatcher, and arm the boot-delay timer. */
void diag_init(void);

/**
 * @brief  Run the UDS dispatcher on one request.
 * @param  req       raw UDS request bytes (SID + parameters)
 * @param  req_len   request length
 * @param  resp      response buffer (at least UDS_MAX_RESPONSE_SIZE = 64 bytes)
 * @param  resp_len  output: number of response bytes written (0 = no response produced)
 * @retval number of response bytes (>0 = response produced), 0 = none
 */
int diag_dispatch(const uint8_t *req, size_t req_len,
                  uint8_t *resp, size_t *resp_len);

#ifdef __cplusplus
}
#endif

#endif /* DIAG_BRIDGE_H */
