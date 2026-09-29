/*
 * socketcan_transport.h - Mode B transport: UDS requests over ISO-TP/SocketCAN.
 * The gateway (tester role) sends requests to the ECU request ID (0x7E0) and
 * reassembles ECU responses from 0x7E8.
 */
#ifndef SOCKETCAN_TRANSPORT_H
#define SOCKETCAN_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

/*
 * Open the CAN interface (e.g. "can0") and bring up the ISO-TP client.
 * Returns 0 on success, -1 on error (socket stays closed).
 */
int can_transport_init(const char *iface);

/*
 * One UDS request/response cycle over CAN (serialized internally).
 * Returns the response length (>0), or -1 on timeout / transport error.
 */
int can_transport_request(const uint8_t *req, size_t req_len,
                          uint8_t *resp, size_t resp_cap, size_t *resp_len,
                          uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* SOCKETCAN_TRANSPORT_H */
