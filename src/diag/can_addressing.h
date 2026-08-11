/**
 * @file    can_addressing.h
 * @brief   CAN addressing layer -- Physical vs Functional address distinction
 * @note    ISO 15765-2 / ISO 14229-1 (11-bit standard ID) based
 *
 * Why needed:
 *   From the received CAN ID alone, we must determine whether this frame is
 *     (1) a physical request targeting this node 1:1 (0x7E0~0x7E7),
 *     (2) a functional broadcast to all ECUs (0x7DF),
 *     (3) unrelated traffic (anything else)
 *   to correctly compute the response ID and apply per-service response suppression policy.
 *
 *   Functional request (0x7DF) response ID = this node's physical response ID (0x7E8).
 *   (Must not simply send to 0x7DF+8=0x7E7.)
 *
 * Multi-ECU extension:
 *   Node physical address is determined at compile time by the 3 macros below.
 *   To use a different ECU slot (e.g. 0x7E2/0x7EA), modify only this header.
 */

#ifndef __CAN_ADDRESSING_H
#define __CAN_ADDRESSING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* === Node CAN IDs (compile-time macros) ===
 * ISO 15765-2 11-bit canonical addressing:
 *   Functional request 0x7DF (broadcast)
 *   Physical request 0x7E0~0x7E7, physical response 0x7E8~0x7EF (one pair per ECU)
 * This node default assignment: request 0x7E0 / response 0x7E8
 */
#define CAN_ID_FUNCTIONAL_REQ    0x7DFU   /**< Functional request (broadcast to all ECUs) */
#define CAN_ID_PHYSICAL_REQ      0x7E0U   /**< This node physical request */
#define CAN_ID_PHYSICAL_RESP     0x7E8U   /**< This node physical response */

/* === Addressing types === */
typedef enum {
    ADDR_IGNORE    = 0,   /**< Not targeting this node -> ignore */
    ADDR_PHYSICAL  = 1,   /**< Physical request (1:1, 0x7E0) */
    ADDR_FUNCTIONAL = 2   /**< Functional request (broadcast, 0x7DF) */
} AddrType_t;

/**
 * @brief  Received CAN ID -> addressing type classification
 * @param  can_id: received standard CAN ID
 * @retval ADDR_PHYSICAL / ADDR_FUNCTIONAL / ADDR_IGNORE
 * @note   Used as primary filter before ISO-TP/UDS processing. Upper layer ignores ADDR_IGNORE.
 */
static inline AddrType_t CAN_Addr_Classify(uint32_t can_id)
{
    if (can_id == CAN_ID_FUNCTIONAL_REQ) {
        return ADDR_FUNCTIONAL;
    }
    if (can_id == CAN_ID_PHYSICAL_REQ) {
        return ADDR_PHYSICAL;
    }
    return ADDR_IGNORE;
}

#ifdef __cplusplus
}
#endif

#endif /* __CAN_ADDRESSING_H */
