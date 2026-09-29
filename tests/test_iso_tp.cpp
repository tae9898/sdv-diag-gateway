/*
 * test_iso_tp.cpp -- hermetic tests for the ported ISO-TP client (src/diag/iso_tp.c).
 * No CAN hardware: this TU provides platform_can_send (link-time injection over
 * the diag_stack static lib) and captures every TX frame; ECU frames are fed
 * back through ISO_TP_ProcessFrame. Timeouts use ISO_TP_Tick's explicit now_ms.
 */
#include <gtest/gtest.h>
#include <cstdint>
#include <string>
#include <vector>

extern "C" {
#include "can_addressing.h"
#include "iso_tp.h"
#include "plat.h"
}

/* ---- Test doubles: TX capture + response capture ---- */

struct TxFrame {
    uint32_t id;
    std::vector<uint8_t> bytes;
};
static std::vector<TxFrame> g_tx;

static std::vector<uint8_t> g_response;
static bool g_got_response = false;

extern "C" int platform_can_send(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    g_tx.push_back({can_id, {data, data + len}});
    return 0;
}

static void response_handler(const uint8_t *data, uint32_t len)
{
    g_response.assign(data, data + len);
    g_got_response = true;
}

/* Fixture: fresh ISO-TP state per test. */
class IsoTpClient : public ::testing::Test {
protected:
    void SetUp() override
    {
        g_tx.clear();
        g_response.clear();
        g_got_response = false;
        ISO_TP_Init();
        ISO_TP_RegisterResponseHandler(response_handler);
    }

    /* Feed a frame as if received from the bus. */
    void rx(uint32_t id, const std::vector<uint8_t>& bytes)
    {
        ISO_TP_ProcessFrame(id, bytes.data(), (uint8_t)bytes.size());
    }

    /* VIN-length (20B) payload: 0x62 F1 90 + 17 chars. */
    static std::vector<uint8_t> vin_response()
    {
        std::vector<uint8_t> v{0x62, 0xF1, 0x90};
        const std::string vin = "WVWZZZ3CZWE000001";
        v.insert(v.end(), vin.begin(), vin.end());
        return v;
    }
};

/* ---- 1. Single-frame request → 0x7E0 ---- */

TEST_F(IsoTpClient, SingleFrameRequest_GoesTo7E0)
{
    std::vector<uint8_t> req{0x22, 0xF1, 0x90};
    ISO_TP_SendRequest(CAN_ID_PHYSICAL_REQ, req.data(), (uint16_t)req.size());

    ASSERT_EQ(g_tx.size(), 1u);
    EXPECT_EQ(g_tx[0].id, 0x7E0u);
    EXPECT_EQ(g_tx[0].bytes, (std::vector<uint8_t>{0x03, 0x22, 0xF1, 0x90}));
}

/* ---- 2. ECU single-frame response → handler (classic SF + FD escape SF) ---- */

TEST_F(IsoTpClient, EcuSingleFrameResponse_Delivered)
{
    rx(0x7E8, {0x03, 0x62, 0xF1, 0x90});
    ASSERT_TRUE(g_got_response);
    EXPECT_EQ(g_response, (std::vector<uint8_t>{0x62, 0xF1, 0x90}));

    /* FD escape SF (0x00 len …) — the path the ECU's 20-byte VIN takes. */
    g_got_response = false;
    std::vector<uint8_t> sf{0x00, 0x14};
    auto vin = vin_response();
    sf.insert(sf.end(), vin.begin(), vin.end());
    rx(0x7E8, sf);
    ASSERT_TRUE(g_got_response);
    EXPECT_EQ(g_response.size(), 20u);
    EXPECT_EQ(g_response, vin);
}

/* ---- 3. ECU multi-frame response: FC to 0x7E0 + reassembly ---- */

TEST_F(IsoTpClient, EcuMultiFrameResponse_SendsFcTo7E0AndReassembles)
{
    /* FF: total=100 (0x064), 62 payload bytes → we owe an FC. */
    std::vector<uint8_t> ff(64, 0xAA);
    ff[0] = 0x10;               /* PCI + length high nibble (0) */
    ff[1] = 100;                /* length low byte */
    for (int i = 0; i < 62; i++) ff[2 + i] = (uint8_t)i;
    rx(0x7E8, ff);

    /* Client-role FC must go to the ECU request ID (0x7E8 - 8 = 0x7E0). */
    ASSERT_EQ(g_tx.size(), 1u);
    EXPECT_EQ(g_tx[0].id, 0x7E0u);
    EXPECT_EQ(g_tx[0].bytes.size(), 16u);  /* FC frames are DLC-16 padded */
    EXPECT_EQ(g_tx[0].bytes[0], 0x30u);    /* CTS */
    EXPECT_EQ(g_tx[0].bytes[1], 0x00u);    /* BS = 0 (unlimited) */
    EXPECT_EQ(g_tx[0].bytes[2], 0x05u);    /* STmin = 5ms */

    /* CF seq 1 with the remaining 38 bytes completes the message. */
    std::vector<uint8_t> cf(39, 0xBB);
    cf[0] = 0x21;
    for (int i = 0; i < 38; i++) cf[1 + i] = (uint8_t)(62 + i);
    rx(0x7E8, cf);

    ASSERT_TRUE(g_got_response);
    ASSERT_EQ(g_response.size(), 100u);
    for (int i = 0; i < 100; i++) EXPECT_EQ(g_response[i], (uint8_t)i) << "byte " << i;
}

/* ---- 4. Foreign CAN IDs ignored ---- */

TEST_F(IsoTpClient, ForeignCanId_Ignored)
{
    rx(0x7E0, {0x03, 0x62, 0xF1, 0x90});   /* request ID: not ours (client role) */
    rx(0x7DF, {0x03, 0x62, 0xF1, 0x90});   /* functional broadcast: not ours */
    EXPECT_FALSE(g_got_response);
    EXPECT_TRUE(g_tx.empty());             /* no FC, nothing */
}

/* ---- 5. Multi-frame request: FF, then CFs driven by ECU's FC ---- */

TEST_F(IsoTpClient, MultiFrameRequest_FfThenCfsAfterFc)
{
    std::vector<uint8_t> req(130);
    for (int i = 0; i < 130; i++) req[i] = (uint8_t)i;
    ISO_TP_SendRequest(CAN_ID_PHYSICAL_REQ, req.data(), 130);

    /* FF: {0x10, 0x82} + first 62 request bytes, DLC 64. */
    ASSERT_EQ(g_tx.size(), 1u);
    EXPECT_EQ(g_tx[0].id, 0x7E0u);
    EXPECT_EQ(g_tx[0].bytes.size(), 64u);
    EXPECT_EQ(g_tx[0].bytes[0], 0x10u);
    EXPECT_EQ(g_tx[0].bytes[1], 130u);      /* 0x82 */
    EXPECT_EQ(g_tx[0].bytes[2], 0x00u);     /* req[0] */
    EXPECT_EQ(g_tx[0].bytes[63], 61u);      /* req[61] */

    /* ECU's FC (CTS, BS=0, STmin=0) releases CF seq 1 immediately. */
    rx(0x7E8, {0x30, 0x00, 0x00});
    ASSERT_EQ(g_tx.size(), 2u);
    EXPECT_EQ(g_tx[1].bytes[0], 0x21u);     /* CF seq 1 */
    EXPECT_EQ(g_tx[1].bytes.size(), 64u);   /* 63 payload + PCI */
    EXPECT_EQ(g_tx[1].bytes[1], 62u);       /* req[62] */

    /* Final CF seq 2 after one Tick (STmin=0 → immediate). */
    ISO_TP_Tick(platform_get_tick_ms());
    ASSERT_EQ(g_tx.size(), 3u);
    EXPECT_EQ(g_tx[2].bytes[0], 0x22u);     /* CF seq 2 */
    EXPECT_EQ(g_tx[2].bytes.size(), 6u);    /* 5 payload + PCI */
    EXPECT_EQ(g_tx[2].bytes[1], 125u);      /* req[125] */
}

/* ---- 6. No FC → TX timeout frees the channel ---- */

TEST_F(IsoTpClient, NoFc_TxTimeoutFreesChannel)
{
    std::vector<uint8_t> req(130, 0x2E);
    ISO_TP_SendRequest(CAN_ID_PHYSICAL_REQ, req.data(), 130);   /* FF sent, WAIT_FC */
    ASSERT_EQ(g_tx.size(), 1u);

    ISO_TP_Tick(platform_get_tick_ms() + 1100);                  /* N_Bs = 1000ms */

    std::vector<uint8_t> sf{0x22, 0xF1, 0x90};
    ISO_TP_SendRequest(CAN_ID_PHYSICAL_REQ, sf.data(), (uint16_t)sf.size());
    ASSERT_EQ(g_tx.size(), 2u);                                  /* not "TX busy"-dropped */
    EXPECT_EQ(g_tx[1].bytes, (std::vector<uint8_t>{0x03, 0x22, 0xF1, 0x90}));
}

/* ---- 7. Missing CF → RX timeout aborts reassembly ---- */

TEST_F(IsoTpClient, MissingCf_RxTimeoutAborts)
{
    std::vector<uint8_t> ff(64, 0xAA);
    ff[0] = 0x10; ff[1] = 100;
    rx(0x7E8, ff);                          /* WAIT_CF (FC captured) */

    ISO_TP_Tick(platform_get_tick_ms() + 1100);                  /* N_Cr = 1000ms */

    std::vector<uint8_t> cf(39, 0xBB);
    cf[0] = 0x21;
    rx(0x7E8, cf);                          /* too late → ignored */
    EXPECT_FALSE(g_got_response);
}

/* ---- 8. CF sequence mismatch aborts reassembly ---- */

TEST_F(IsoTpClient, CfSeqMismatch_AbortsReassembly)
{
    std::vector<uint8_t> ff(64, 0xAA);
    ff[0] = 0x10; ff[1] = 100;
    rx(0x7E8, ff);                          /* expect CF seq 1 next */

    std::vector<uint8_t> cf(39, 0xBB);
    cf[0] = 0x22;                           /* seq 2: mismatch → abort */
    rx(0x7E8, cf);

    cf[0] = 0x23;                           /* further CFs also ignored */
    rx(0x7E8, cf);
    EXPECT_FALSE(g_got_response);
}
