/*
 * test_uds_dispatcher.cpp -- GoogleTest regression suite for the in-process C UDS
 * engine (diag_dispatch API). Links diag_stack + diag_bridge only (no vsomeip).
 */
#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
#include <vector>

extern "C" {
#include "diag_bridge.h"
}

/* Fixture: fresh diag_init() per test so session/timer state is clean. */
class UdsDispatcher : public ::testing::Test {
protected:
    void SetUp() override { diag_init(); }

    /* Dispatch req bytes, return response vector + length. */
    std::vector<uint8_t> run(const std::vector<uint8_t>& req) {
        uint8_t buf[DIAG_RESP_MAX];
        size_t len = 0;
        int rc = diag_dispatch(req.data(), req.size(), buf, &len);
        return {buf, buf + rc};
    }
};

/* ---- 0x22 ReadDataByIdentifier ---- */

TEST_F(UdsDispatcher, ReadVin) {
    auto r = run({0x22, 0xF1, 0x90});
    ASSERT_EQ(r.size(), 20u);
    EXPECT_EQ(r[0], 0x62u); EXPECT_EQ(r[1], 0xF1u); EXPECT_EQ(r[2], 0x90u);
    std::string vin(r.begin() + 3, r.end());
    EXPECT_EQ(vin, "WVWZZZ3CZWE000001");
}

TEST_F(UdsDispatcher, ReadHwVersion) {
    auto r = run({0x22, 0xF1, 0x93});
    ASSERT_GE(r.size(), 3u);
    EXPECT_EQ(r[0], 0x62u); EXPECT_EQ(r[1], 0xF1u); EXPECT_EQ(r[2], 0x93u);
    std::string hw(r.begin() + 3, r.end());
    EXPECT_EQ(hw, "HW Rev1.0");
}

TEST_F(UdsDispatcher, ReadUnknownDid_NrcOutOfRange) {
    auto r = run({0x22, 0xFF, 0xFF});
    ASSERT_EQ(r.size(), 3u);
    EXPECT_EQ(r[0], 0x7Fu); EXPECT_EQ(r[1], 0x22u); EXPECT_EQ(r[2], 0x31u);
}

/* ---- 0x10 DiagnosticSessionControl ---- */

TEST_F(UdsDispatcher, SessionControl_Extended) {
    auto r = run({0x10, 0x03});
    ASSERT_EQ(r.size(), 6u);
    EXPECT_EQ(r[0], 0x50u); EXPECT_EQ(r[1], 0x03u);
    /* P2=50ms (0x0032), P2*=5000/10=0x01F4 */
    EXPECT_EQ(r[2], 0x00u); EXPECT_EQ(r[3], 0x32u);
    EXPECT_EQ(r[4], 0x01u); EXPECT_EQ(r[5], 0xF4u);
}

/* ---- 0x3E TesterPresent ---- */

TEST_F(UdsDispatcher, TesterPresent) {
    auto r = run({0x3E, 0x00});
    ASSERT_EQ(r.size(), 2u);
    EXPECT_EQ(r[0], 0x7Eu); EXPECT_EQ(r[1], 0x00u);
}

/* ---- OBD-II gating (service-not-supported) ---- */

TEST_F(UdsDispatcher, Obd2Mode01_NotSupported) {
    auto r = run({0x01, 0x00});
    ASSERT_EQ(r.size(), 3u);
    EXPECT_EQ(r[0], 0x7Fu); EXPECT_EQ(r[1], 0x01u); EXPECT_EQ(r[2], 0x11u);
}

TEST_F(UdsDispatcher, Obd2Mode03_NotSupported) {
    auto r = run({0x03, 0x00});
    ASSERT_EQ(r.size(), 3u);
    EXPECT_EQ(r[0], 0x7Fu); EXPECT_EQ(r[1], 0x03u); EXPECT_EQ(r[2], 0x11u);
}

/* ---- OTA stub (0x34) ---- */
/*
 * In default session, CheckAccess(0x34) requires Extended + Security unlock
 * → NRC 0x33 (securityAccessDenied). The RAUC-stub erase path (NRC 0x72) is
 * only reachable in Extended+Unlocked session, so we test the gate here.
 */
TEST_F(UdsDispatcher, Ota_RequestDownload_DefaultSession_NrcSecurityDenied) {
    auto r = run({0x34, 0x44, 0x08, 0x01, 0xE0, 0x00, 0x00, 0x00, 0x02, 0x00});
    ASSERT_EQ(r.size(), 3u);
    EXPECT_EQ(r[0], 0x7Fu); EXPECT_EQ(r[1], 0x34u); EXPECT_EQ(r[2], 0x33u);
}
