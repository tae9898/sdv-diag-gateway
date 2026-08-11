/**
 * @file    diag_session.c
 * @brief   UDS diagnostic session manager implementation
 * @note    Session transitions, S3 timeout, Seed-Key simulation
 *          Ported from obd-simulator: HAL_GetTick() -> platform_get_tick_ms(),
 *          Debug_Print() -> platform_log().
 */

#include "diag_session.h"
#include "uds_service.h"
#include "plat.h"
#include <string.h>

/* === Security failure limits === */
#define DIAG_MAX_FAIL_ATTEMPTS   3U     /**< Maximum Key failure count */
#define DIAG_LOCKOUT_TIME_MS     10000U /**< Lockout time after failure (10s) */

/* === Session control block === */
static Diag_Session_t s_session;
static uint8_t  s_fail_count = 0U;
static uint32_t s_lockout_start = 0U;
static uint32_t s_boot_tick = 0U;           /**< Boot time (boot delay reference point) */

/* === Internal functions === */
static uint16_t compute_key(uint16_t seed);

void DiagSession_Init(void)
{
    (void)memset(&s_session, 0, sizeof(s_session));
    s_session.session_type = DIAG_SESSION_DEFAULT;
    s_session.security_level = DIAG_SEC_LOCKED;
    s_session.last_activity_tick = platform_get_tick_ms();
    s_fail_count = 0U;
    /* s_boot_tick is set in MarkBootReady() (communication ready point).
     * Setting it here is too early (before scheduler start), making boot delay meaningless. */
    platform_log("[DIAG] Session manager init OK\r\n");
}

void DiagSession_MarkBootReady(void)
{
    s_boot_tick = platform_get_tick_ms();
    platform_log("[DIAG] Boot ready — security boot-delay armed\r\n");
}

int DiagSession_SetSession(uint8_t session_type)
{
    if (session_type != DIAG_SESSION_DEFAULT &&
        session_type != DIAG_SESSION_PROGRAMMING &&
        session_type != DIAG_SESSION_EXTENDED) {
        return -1;
    }

    uint8_t prev = s_session.session_type;
    s_session.session_type = session_type;
    s_session.last_activity_tick = platform_get_tick_ms();

    /* Returning to Default also locks security */
    if (session_type == DIAG_SESSION_DEFAULT) {
        s_session.security_level = DIAG_SEC_LOCKED;
    }

    platform_log("[DIAG] Session: %u -> %u\r\n", prev, session_type);
    return 0;
}

/**
 * @brief  Generate seed
 * @note   Uses monotonic tick as entropy source.
 *         Real ECU uses TRNG (True Random Number Generator).
 *         Tick is sufficient for simulation.
 */
uint16_t DiagSession_GenerateSeed(void)
{
    uint32_t tick = platform_get_tick_ms();
    s_session.seed = (uint16_t)((tick ^ (tick >> 16U)) & 0xFFFFU);

    if (s_session.seed == 0U) {
        s_session.seed = 0x0001U;  /* 0 implies already unlocked, so forbidden */
    }

    s_session.seed_is_fresh = 1U;
    s_session.last_activity_tick = platform_get_tick_ms();

    platform_log("[DIAG] Seed: 0x%04X\r\n", s_session.seed);
    return s_session.seed;
}

/**
 * @brief  Key verification (returns result by reason)
 * @retval DIAG_KEY_OK / DIAG_KEY_INVALID / DIAG_KEY_EXCEEDED_ATTEMPTS /
 *         DIAG_KEY_DELAY_NOT_EXPIRED
 *
 * Security procedure (priority order):
 *   1. Boot delay not elapsed       -> DELAY_NOT_EXPIRED (NRC 0x37)
 *   2. 3 failures then lockout     -> EXCEEDED_ATTEMPTS   (NRC 0x36)
 *   3. Seed not issued             -> INVALID             (NRC 0x35)
 *   4. Compare key = compute_key(seed) -> OK (unlock) / INVALID (counter++)
 *
 * @note   During lockout/delay, even the correct key is rejected (ISO 14229-1).
 *         The previous structure (int 0/-1) only returned NRC 0x35 (InvalidKey)
 *         during lockout, which did not provide the attacker the standard signal
 *         (0x36/0x37) indicating "currently locked".
 */
DiagSecGate_t DiagSession_CheckSecurityGate(void)
{
    uint32_t now = platform_get_tick_ms();

    /* Boot delay (NRC 0x37) */
    if ((now - s_boot_tick) < DIAG_BOOT_DELAY_MS) {
        platform_log("[DIAG] SecGate: boot delay\r\n");
        return DIAG_SEC_GATE_DELAY;
    }
    /* Attempt limit lockout (NRC 0x36). Auto-reset on expiry. */
    if (s_fail_count >= DIAG_MAX_FAIL_ATTEMPTS) {
        if ((now - s_lockout_start) < DIAG_LOCKOUT_TIME_MS) {
            platform_log("[DIAG] SecGate: locked\r\n");
            return DIAG_SEC_GATE_LOCKED;
        }
        s_fail_count = 0U;  /* Lockout expired -> reset */
    }
    return DIAG_SEC_GATE_OK;
}

DiagKeyResult_t DiagSession_VerifyKey(uint16_t key)
{
    /* 1. Gate (boot delay/lockout) -- shared with requestSeed (M2) */
    switch (DiagSession_CheckSecurityGate()) {
        case DIAG_SEC_GATE_DELAY:  return DIAG_KEY_DELAY_NOT_EXPIRED;
        case DIAG_SEC_GATE_LOCKED: return DIAG_KEY_EXCEEDED_ATTEMPTS;
        default:                   break;
    }

    /* 2. Seed validity */
    if (!s_session.seed_is_fresh) {
        platform_log("[DIAG] Seed not fresh\r\n");
        return DIAG_KEY_INVALID;
    }

    uint16_t expected = compute_key(s_session.seed);
    s_session.seed_is_fresh = 0U;  /* Seed is single-use */

    /* 3. Compare -> success (unlock) / failure (counter++) */
    if (key == expected) {
        s_session.security_level = DIAG_SEC_LEVEL1;
        s_fail_count = 0U;
        s_session.last_activity_tick = platform_get_tick_ms();
        platform_log("[DIAG] Security unlocked\r\n");
        return DIAG_KEY_OK;
    }

    s_fail_count++;
    platform_log("[DIAG] Key mismatch: got=0x%04X exp=0x%04X (fail %u/%u)\r\n",
                key, expected, s_fail_count, DIAG_MAX_FAIL_ATTEMPTS);

    if (s_fail_count >= DIAG_MAX_FAIL_ATTEMPTS) {
        s_lockout_start = platform_get_tick_ms();
        platform_log("[DIAG] Lockout for %us\r\n", DIAG_LOCKOUT_TIME_MS / 1000U);
    }

    return DIAG_KEY_INVALID;
}

void DiagSession_ResetS3Timeout(void)
{
    s_session.last_activity_tick = platform_get_tick_ms();
}

/**
 * @brief  S3 timeout handling
 * @note   If no activity for 5 seconds, returns to Default session.
 *         No timeout in Default session (already in default state).
 */
void DiagSession_Tick(uint32_t now_ms)
{
    if (s_session.session_type == DIAG_SESSION_DEFAULT) {
        return;
    }

    if ((now_ms - s_session.last_activity_tick) >= DIAG_S3_TIMEOUT_MS) {
        platform_log("[DIAG] S3 timeout -> Default\r\n");
        DiagSession_SetSession(DIAG_SESSION_DEFAULT);
    }
}

/**
 * @brief  Service access permission check
 * @note   SID 0x31 (RoutineControl) rules:
 *         - Extended session required
 *         - Security must be unlocked
 *         If either condition is not met -> NRC 0x33 (SecurityAccessDenied)
 */
/* === Access control policy table (ISO 14229-1 based, 3.3) ===
 * Only listed SIDs are restricted; all others (0x10/0x11/0x22/0x27/0x19/0x3E
 * read/query services) are allowed in all sessions.
 * Write/control services require Extended (+security).
 * Policy changes are made in this one table. */
typedef struct {
    uint8_t sid;            /**< Service ID */
    uint8_t need_extended;  /**< 1 = Extended session required */
    uint8_t need_security;  /**< 1 = SecurityAccess unlock required */
} svc_access_t;

static const svc_access_t k_access_rules[] = {
    { UDS_SID_ROUTINE_CONTROL,       1U, 1U },  /* 0x31 */
    { UDS_SID_WRITE_DATA_BY_ID,      1U, 1U },  /* 0x2E */
    { UDS_SID_IO_CONTROL_BY_ID,      1U, 1U },  /* 0x2F */
    { UDS_SID_COMMUNICATION_CONTROL, 1U, 0U },  /* 0x28: Extended only */
    { UDS_SID_REQUEST_DOWNLOAD,      1U, 1U },  /* 0x34: OTA */
    { UDS_SID_TRANSFER_DATA,         1U, 1U },  /* 0x36: OTA */
    { UDS_SID_REQUEST_TRANSFER_EXIT, 1U, 1U },  /* 0x37: OTA */
};

int DiagSession_CheckAccess(uint8_t sid)
{
    for (uint8_t i = 0U;
         i < (uint8_t)(sizeof(k_access_rules) / sizeof(k_access_rules[0]));
         i++) {
        if (k_access_rules[i].sid == sid) {
            if ((k_access_rules[i].need_extended != 0U) &&
                (s_session.session_type != DIAG_SESSION_EXTENDED)) {
                return -1;
            }
            if ((k_access_rules[i].need_security != 0U) &&
                (s_session.security_level == DIAG_SEC_LOCKED)) {
                return -1;
            }
            return 0;
        }
    }
    return 0;  /* No rule = allowed (read/query services) */
}

/**
 * @brief  Seed-Key algorithm
 * @note   key = ((seed ^ 0x5A3C) rotate_left 3) & 0xFFFF
 *
 *         Simple algorithm for simulation.
 *         Production uses stricter memory protection, RNG quality, etc.
 *
 *         Calculation example: seed = 0xA3F7
 *           1. XOR:  0xA3F7 ^ 0x5A3C = 0xF9CB
 *           2. ROL3: 0xF9CB <<< 3 = (0xF9CB << 3) | (0xF9CB >> 13)
 *                         = 0xFCE58 & 0xFFFF | 0x1F
 *                         = 0xCE5D
 */
static uint16_t compute_key(uint16_t seed)
{
    uint16_t xored = seed ^ DIAG_SEED_XOR_MASK;
    uint16_t rotated = (uint16_t)((xored << 3U) | (xored >> (16U - 3U)));
    return rotated & 0xFFFFU;
}
