/**
 * @file    uds_service.h
 * @brief   UDS (ISO 14229) service dispatcher header
 * @note    5 UDS services + OBD-II Mode 01 legacy bridge
 *
 * UDS Service ID (SID):
 *   First byte is SID. Response is SID + 0x40.
 *   Error: 0x7F + SID + NRC.
 *
 *   Example: request [0x22 0xF1 0x90] (read VIN)
 *            response [0x62 0xF1 0x90 W V W ...]  (0x22 + 0x40 = 0x62)
 *            error    [0x7F 0x22 0x31]            (NRC 0x31 = out of range)
 */

#ifndef __UDS_SERVICE_H
#define __UDS_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "can_addressing.h"   /* AddrType_t */

/* === UDS Service IDs === */
#define UDS_SID_DIAG_SESSION_CTRL    0x10U  /**< DiagnosticSessionControl */
#define UDS_SID_ECU_RESET           0x11U  /**< ECU Reset */
#define UDS_SID_READ_DATA_BY_ID     0x22U  /**< ReadDataByIdentifier */
#define UDS_SID_SECURITY_ACCESS     0x27U  /**< SecurityAccess */
#define UDS_SID_ROUTINE_CONTROL     0x31U  /**< RoutineControl */
#define UDS_SID_TESTER_PRESENT      0x3EU  /**< TesterPresent (S3 keep-alive) */
#define UDS_SID_WRITE_DATA_BY_ID    0x2EU  /**< WriteDataByIdentifier */
#define UDS_SID_COMMUNICATION_CONTROL 0x28U /**< CommunicationControl */
#define UDS_SID_READ_DTC_INFORMATION  0x19U /**< ReadDTCInformation */
#define UDS_SID_IO_CONTROL_BY_ID      0x2FU /**< InputOutputControlByIdentifier */
#define UDS_SID_REQUEST_DOWNLOAD      0x34U /**< RequestDownload (OTA) */
#define UDS_SID_TRANSFER_DATA         0x36U /**< TransferData (OTA) */
#define UDS_SID_REQUEST_TRANSFER_EXIT 0x37U /**< RequestTransferExit (OTA) */
/* OBD-II services (ISO 15031-5 / SAE J1979) -- functional (0x7DF) response eligible.
 * Gateway: SIDs 0x01/0x03/0x04/0x07/0x09 are NOT serviced (no OBD-II simulator);
 * the dispatcher falls through to NRC 0x11 (service-not-supported). */
#define UDS_SID_OBD2_CURRENT_DATA   0x01U  /**< Mode 01: current data */
#define UDS_SID_OBD2_STORED_DTC     0x03U  /**< Mode 03: stored DTC */
#define UDS_SID_OBD2_CLEAR_DTC      0x04U  /**< Mode 04: clear DTC */
#define UDS_SID_OBD2_PENDING_DTC    0x07U  /**< Mode 07: pending DTC */
#define UDS_SID_OBD2_VEHICLE_INFO   0x09U  /**< Mode 09: vehicle information */

/* === Response SID offset === */
#define UDS_RESPONSE_SID_OFFSET     0x40U  /**< response SID = request SID + 0x40 */

/* === NRC (Negative Response Codes) === */
/**
 * NRC is the UDS error code.
 * Negative response format: [0x7F, SID, NRC]
 */
#define NRC_SERVICE_NOT_SUPPORTED   0x11U  /**< service not supported */
#define NRC_SUB_FUNC_NOT_SUPPORTED  0x12U  /**< subfunction not supported */
#define NRC_INCORRECT_MSG_LEN       0x13U  /**< incorrect message length */
#define NRC_CONDITIONS_NOT_CORRECT  0x22U  /**< conditions not correct */
#define NRC_REQUEST_OUT_OF_RANGE    0x31U  /**< request out of range */
#define NRC_SECURITY_ACCESS_DENIED  0x33U  /**< security access denied */
#define NRC_INVALID_KEY             0x35U  /**< invalid key */
#define NRC_EXCEEDED_ATTEMPTS       0x36U  /**< exceeded number of attempts (locked) */
#define NRC_REQUIRED_TIME_DELAY         0x37U  /**< required time delay not expired (boot/delay) */
#define NRC_TRANSFER_DATA_SUSPENDED     0x71U  /**< TransferData suspended (size exceeded etc.) */
#define NRC_GENERAL_PROGRAMMING_FAILURE 0x72U  /**< flash write failure */
#define NRC_WRONG_BLOCK_SEQUENCE        0x73U  /**< blockSequenceCounter mismatch */

/* === ECU Reset subfunctions === */
#define UDS_RESET_HARD              0x01U  /**< hard reset (full restart) */
#define UDS_RESET_SOFT              0x03U  /**< soft reset (initialize) */

/* === DID (Data Identifier) === */
#define UDS_DID_VIN                 0xF190U  /**< vehicle identification number */
#define UDS_DID_HW_VERSION          0xF193U  /**< hardware version */
#define UDS_DID_SW_VERSION          0xF195U  /**< software version */
#define UDS_DID_ECU_NAME            0xF198U  /**< ECU name */
#define UDS_DID_IO_CONTROL          0x0200U  /**< virtual IO port (for 0x2F control) */

/* === Routine Control === */
#define UDS_ROUTINE_START           0x01U
#define UDS_ROUTINE_STOP            0x02U
#define UDS_ROUTINE_REQUEST_RESULT  0x03U
#define UDS_ROUTINE_ID_DTC_CLEAR    0x0201U  /**< DTC clear */
#define UDS_ROUTINE_ID_SELF_TEST    0x0202U  /**< self test */

/* === Maximum response size === */
#define UDS_MAX_RESPONSE_SIZE       64U

/* === P2/P2* server timing (ISO 14229-1, DiagnosticSessionControl response) ===
 * P2  : max delay from request reception to response transmission (ms, 1ms resolution)
 * P2* : extended delay after NRC 0x78 (ResponsePending) (ms; response field is 10ms resolution)
 * Simulator responds immediately, so P2=50ms is easily met. P2* is reference value
 * since 0x78 is not implemented.
 */
#define UDS_P2_SERVER_MAX_MS        50U     /* P2  -> 0x0032 (1ms resolution) */
#define UDS_P2_STAR_SERVER_MAX_MS   5000U   /* P2* -> 0x01F4 (5000ms/10, 10ms resolution) */

/* === API === */

void UDS_Init(void);

/**
 * @brief  UDS request dispatch
 * @param  request:      request data (SID + parameters)
 * @param  request_len:  request length
 * @param  addr:         received addressing type (Physical/Functional)
 * @param  response:     response buffer
 * @param  response_len: response length (output, 0=no response)
 * @note   Called when ISO-TP reassembly is complete.
 *         If functional request (addr==ADDR_FUNCTIONAL) and the SID does not support
 *         functional response, response is suppressed (*response_len=0) (ISO 14229-1).
 */
void UDS_DispatchRequest(const uint8_t *request, uint16_t request_len,
                         AddrType_t addr,
                         uint8_t *response, uint16_t *response_len);

/**
 * @brief  Check if SID is eligible for functional addressing (0x7DF) response
 * @note   Only implemented OBD-II core services (0x01/0x03/0x04/0x07/0x09, ISO 15031-5)
 *         allow functional response. UDS diagnostic services (0x10/0x11/0x22/0x27/0x31)
 *         are physical-only (safety policy: prevent broadcast session/reset).
 *         Unimplemented OBD-II modes (0x02/0x05/0x06/0x08/0x0A) also excluded --
 *         functional requests must not receive negative responses, so filter here
 *         to suppress response. Policy adjustments in this single table.
 */
uint8_t UDS_IsFunctionallyAddressable(uint8_t sid);

/* === External variables ===
 * NOTE: defined in diag_bridge.cpp (C linkage). The gateway satisfies this extern
 * instead of the firmware's main.c. */
extern volatile uint8_t g_soft_reset_requested;

#ifdef __cplusplus
}
#endif

#endif /* __UDS_SERVICE_H */
