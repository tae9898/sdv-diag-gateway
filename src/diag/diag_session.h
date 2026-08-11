/**
 * @file    diag_session.h
 * @brief   UDS diagnostic session manager header
 * @note    Session types, security access, S3 timeout management
 *
 * Why sessions are needed:
 *   In UDS, allowed services depend on the current session state.
 *   - Default session: basic read-only (OBD-II PID queries, etc.)
 *   - Extended session: advanced diagnostics (routine execution, actuator control)
 *   - Programming session: firmware update
 *
 *   S3 timeout: if no activity for 5 seconds, automatically returns to Default session
 *   (Safety mechanism: if the diagnostic tool disconnects, ECU returns to normal state)
 */

#ifndef __DIAG_SESSION_H
#define __DIAG_SESSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* === Session types === */
#define DIAG_SESSION_DEFAULT       0x01U  /**< Default session (read-only) */
#define DIAG_SESSION_PROGRAMMING   0x02U  /**< Programming session (FW update) */
#define DIAG_SESSION_EXTENDED      0x03U  /**< Extended session (advanced diagnostics) */

/* === Security levels === */
#define DIAG_SEC_LOCKED            0x00U  /**< Locked state */
#define DIAG_SEC_LEVEL1            0x01U  /**< Unlocked state */

/* === S3 timeout === */
/**
 * S3 timeout: 5 seconds
 * Meaning: if no UDS request for 5 seconds, returns to Default session
 * Reason: ECU returns to safe state even if diagnostic tool terminates abnormally
 */
#define DIAG_S3_TIMEOUT_MS         5000U

/* === Seed-Key simulation === */
#define DIAG_SEED_XOR_MASK         0x5A3CU

/* === SecurityAccess brute-force mitigation === */
/**
 * Rejects 0x27 (sendKey) verification for a short time after boot (NRC 0x37).
 * Minimum mitigation against power-on brute-force attacks. See ISO 14229.
 */
#define DIAG_BOOT_DELAY_MS         1000U

/* === Key verification results -- handler selects NRC by distinguishing reason ===
 * Extended from previous int (0/-1) return: when locked or delay not elapsed,
 * returns ISO 14229-1 standard NRC (0x36/0x37) instead of just "invalid key".
 */
typedef enum {
    DIAG_KEY_OK = 0,              /**< Success (unlock) */
    DIAG_KEY_INVALID,            /**< Key mismatch / seed not issued -> NRC 0x35 */
    DIAG_KEY_EXCEEDED_ATTEMPTS,  /**< Attempt exceeded / locked -> NRC 0x36 */
    DIAG_KEY_DELAY_NOT_EXPIRED   /**< Boot/delay not elapsed -> NRC 0x37 */
} DiagKeyResult_t;

/* SecurityAccess gate (boot delay/lockout) -- shared by requestSeed/sendKey (M2) */
typedef enum {
    DIAG_SEC_GATE_OK = 0,
    DIAG_SEC_GATE_DELAY,   /**< Boot delay -> NRC 0x37 */
    DIAG_SEC_GATE_LOCKED   /**< Locked -> NRC 0x36 */
} DiagSecGate_t;

/* === Session control block === */
typedef struct {
    uint8_t  session_type;          /**< Current session type */
    uint8_t  security_level;        /**< Security level (LOCKED/LEVEL1) */
    uint16_t seed;                  /**< Last generated seed */
    uint8_t  seed_is_fresh;         /**< Seed not yet used (prevents reuse) */
    uint32_t last_activity_tick;    /**< Last activity time (ms) */
} Diag_Session_t;

/* === API === */

void DiagSession_Init(void);

/**
 * @brief  Sets the communication-ready point as the SecurityAccess boot-delay reference
 * @note   DiagSession_Init() runs before scheduler start (before clock/GPIO/FDCAN init),
 *         which is too early -- if actual boot exceeds 1 second, boot delay is already
 *         expired and meaningless.
 *         Must be called from main() after FDCAN start and diagnostic communication is ready.
 */
void DiagSession_MarkBootReady(void);

int  DiagSession_SetSession(uint8_t session_type);

uint16_t DiagSession_GenerateSeed(void);

/**
 * @brief  Key verification (returns result by reason)
 * @retval DIAG_KEY_OK / DIAG_KEY_INVALID / DIAG_KEY_EXCEEDED_ATTEMPTS /
 *         DIAG_KEY_DELAY_NOT_EXPIRED
 * @note   Handler selects NRC 0x00 (positive)/0x35/0x36/0x37 based on return value.
 *         Within DIAG_BOOT_DELAY_MS after boot, or during 3-failure lockout period,
 *         returns EXCEEDED/DELAY regardless of key content.
 */
DiagKeyResult_t DiagSession_VerifyKey(uint16_t key);

/**
 * @brief  SecurityAccess gate (boot delay/lockout) common check (M2)
 * @retval OK / DELAY(0x37) / LOCKED(0x36). Auto-reset on lockout expiry.
 * @note   Called from both requestSeed and sendKey -- prevents bypass by pre-fetching seed only.
 */
DiagSecGate_t DiagSession_CheckSecurityGate(void);

void DiagSession_ResetS3Timeout(void);
void DiagSession_Tick(uint32_t now_ms);

/**
 * @brief  Service access permission check
 * @retval 0: allowed, -1: denied
 * @note   SID 0x31 requires Extended + Security Unlock
 */
int  DiagSession_CheckAccess(uint8_t sid);

#ifdef __cplusplus
}
#endif

#endif /* __DIAG_SESSION_H */
