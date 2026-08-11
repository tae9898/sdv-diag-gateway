/**
 * @file    vehicle_config.h
 * @brief   ECU identity information (vehicle/ECU identification) single configuration point
 * @note    UDS ReadDataByIdentifier (DID 0xF190 VIN / 0xF193 HW / 0xF195 SW /
 *          0xF198 ECU name) and OBD-II Mode 09 (VIN) share these identity strings.
 *          Board/build-specific customization in this single header.
 */
#ifndef __VEHICLE_CONFIG_H
#define __VEHICLE_CONFIG_H

/* === Vehicle identification -- ISO 3779 standard 17-digit VIN === */
#define VEHICLE_VIN             "WVWZZZ3CZWE000001"

/* === ECU identity (UDS DID 0xF193/0xF195/0xF198) === */
#define VEHICLE_HW_VERSION      "HW Rev1.0"
#define VEHICLE_SW_VERSION      "SW Phase1.0"
#define ECU_NAME                "OBD-SIM-G431"

#endif /* __VEHICLE_CONFIG_H */
