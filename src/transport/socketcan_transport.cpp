/*
 * socketcan_transport.cpp - Mode B transport: AF_CAN socket driving the
 * ported ISO-TP client (src/diag/iso_tp.c).
 *
 * Threading: vsomeip may deliver requests on multiple worker threads, so
 * can_transport_request serializes the whole ISO-TP transaction with one
 * mutex (the ISO-TP context is static/single-transaction). Requests from
 * concurrent SOME/IP callers queue here rather than interleave on the bus.
 */
#include "socketcan_transport.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <mutex>

#include "can_addressing.h"
#include "iso_tp.h"
#include "plat.h"

static int g_fd = -1;
static std::mutex g_mutex;

/* Response hand-off from the ISO-TP handler (runs on this same thread,
 * from inside the poll loop). */
static struct {
    uint8_t *buf;
    size_t cap;
    size_t len;
    bool done;
} g_rsp;

static void response_handler(const uint8_t *data, uint32_t len)
{
    if (len > g_rsp.cap) {
        /* ponytail: resp capped at caller buffer (64 today, DIAG_RESP_MAX);
         * raise DIAG_RESP_MAX if the ECU ever sends longer responses. */
        platform_log("[CAN] response %lu > cap %lu, truncated\n",
                     (unsigned long)len, (unsigned long)g_rsp.cap);
        len = (uint32_t)g_rsp.cap;
    }
    memcpy(g_rsp.buf, data, len);
    g_rsp.len = len;
    g_rsp.done = true;
}

/* ISO-TP TX hook (plat.h). The kernel rounds len up to a valid FD DLC,
 * exactly as the ECU firmware's FDCAN_BytesToDlc did. */
extern "C" int platform_can_send(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    if (g_fd < 0 || len > CANFD_MAX_DLEN) {
        return -1;
    }
    canfd_frame cf{};
    cf.can_id = can_id;
    cf.len = len;
    cf.flags = CANFD_BRS;  /* match the ECU firmware (FDCAN_BRS_ON, 500k/2M) */
    memcpy(cf.data, data, len);
    return (write(g_fd, &cf, sizeof(cf)) == (ssize_t)sizeof(cf)) ? 0 : -1;
}

int can_transport_init(const char *iface)
{
    ifreq ifr{};
    sockaddr_can addr{};

    g_fd = socket(AF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
    if (g_fd < 0) {
        fprintf(stderr, "can_transport: socket: %s\n", strerror(errno));
        return -1;
    }

    int fd_frames = 1;
    if (setsockopt(g_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &fd_frames, sizeof(fd_frames)) < 0) {
        fprintf(stderr, "can_transport: CAN_RAW_FD_FRAMES: %s\n", strerror(errno));
        goto fail;
    }

    /* Only the ECU response ID reaches ISO_TP_ProcessFrame. */
    struct can_filter filter;
    filter.can_id = CAN_ID_PHYSICAL_RESP;
    filter.can_mask = CAN_SFF_MASK;
    if (setsockopt(g_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
        fprintf(stderr, "can_transport: CAN_RAW_FILTER: %s\n", strerror(errno));
        goto fail;
    }

    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    if (ioctl(g_fd, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr, "can_transport: SIOCGIFINDEX(%s): %s\n", iface, strerror(errno));
        goto fail;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(g_fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "can_transport: bind(%s): %s\n", iface, strerror(errno));
        goto fail;
    }

    ISO_TP_Init();
    ISO_TP_RegisterResponseHandler(response_handler);
    return 0;

fail:
    close(g_fd);
    g_fd = -1;
    return -1;
}

int can_transport_request(const uint8_t *req, size_t req_len,
                          uint8_t *resp, size_t resp_cap, size_t *resp_len,
                          uint32_t timeout_ms)
{
    if (g_fd < 0 || req == nullptr || req_len == 0 || req_len > ISO_TP_MAX_MESSAGE_SIZE) {
        return -1;  /* pre-check: ISO-TP would log-and-drop oversized silently */
    }

    std::lock_guard<std::mutex> lk(g_mutex);

    /* A previously timed-out multi-frame transmit may have left TX in
     * WAIT_FC — clear it so this request is never "TX busy"-dropped. */
    ISO_TP_AbortTx();

    /* Discard stale frames from a previously timed-out request, so a late
     * response is never mistaken for this request's answer. */
    pollfd pfd{g_fd, POLLIN, 0};
    canfd_frame cf;
    while (poll(&pfd, 1, 0) == 1) {
        (void)read(g_fd, &cf, sizeof(cf));
    }

    g_rsp = {resp, resp_cap, 0, false};
    ISO_TP_SendRequest(CAN_ID_PHYSICAL_REQ, req, (uint16_t)req_len);

    const uint32_t deadline = platform_get_tick_ms() + timeout_ms;
    while (!g_rsp.done) {
        int32_t now = (int32_t)platform_get_tick_ms();
        if (now - (int32_t)deadline >= 0) {
            return -1;  /* timeout */
        }
        if (poll(&pfd, 1, 1) == 1) {  /* 1 ms keeps ISO_TP_Tick CF pacing honest */
            ssize_t n = read(g_fd, &cf, sizeof(cf));
            if (n == (ssize_t)sizeof(canfd_frame)) {
                ISO_TP_ProcessFrame(cf.can_id, cf.data, cf.len);
            } else if (n == (ssize_t)sizeof(can_frame)) {
                can_frame *f = (can_frame *)&cf;
                ISO_TP_ProcessFrame(f->can_id, f->data,
                                    f->len > CAN_MAX_DLEN ? CAN_MAX_DLEN : f->len);
            }
        }
        ISO_TP_Tick(platform_get_tick_ms());  /* RX/TX timeouts + STmin pacing */
    }

    *resp_len = g_rsp.len;
    return (int)g_rsp.len;
}
