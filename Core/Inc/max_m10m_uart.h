/**
 ******************************************************************************
 * @file           : max_m10m.h
 * @brief          : MAX-M10M-00B GNSS Module Driver Header
 * @attention
 * Driver for u-blox MAX-M10M-00B GNSS module
 * Communication: UART2 @ 9600 baud, 8 data bits, 1 stop bit, no parity
 * Based on MAX-M10M-00B datasheet (UBX-22028884)
 ******************************************************************************
 */

#ifndef MAX_M10M_H
#define MAX_M10M_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* UBX Protocol Sync Characters */
#define UBX_SYNC_CHAR_1             0xB5
#define UBX_SYNC_CHAR_2             0x62

/* UBX Message Classes */
#define UBX_CLASS_NAV               0x01
#define UBX_CLASS_ACK               0x05
#define UBX_CLASS_CFG               0x06

/* UBX NAV Message IDs */
#define UBX_NAV_PVT                 0x07    /* Navigation Position Velocity Time */
#define UBX_NAV_STATUS              0x03    /* Receiver Navigation Status */

/* UBX CFG Message IDs */
#define UBX_CFG_PRT                 0x00    /* Port Configuration */
#define UBX_CFG_RATE                0x08    /* Measurement Rate Settings */
#define UBX_CFG_MSG                 0x01    /* Message Rate Configuration */

/* UBX ACK Message IDs */
#define UBX_ACK_ACK                 0x01    /* Acknowledge */
#define UBX_ACK_NAK                 0x00    /* Not Acknowledge */

/* Buffer Sizes */
#define MAX_M10M_RX_BUFFER_SIZE     256
#define MAX_M10M_TX_BUFFER_SIZE     128

/* GNSS Module States */
typedef enum {
    GNSS_STATE_UNINITIALIZED = 0,
    GNSS_STATE_INITIALIZING,
    GNSS_STATE_READY,
    GNSS_STATE_NO_FIX,
    GNSS_STATE_FIX_ACQUIRED,
    GNSS_STATE_ERROR
} gnss_state_t;

/* GPS Fix Types */
typedef enum {
    FIX_TYPE_NO_FIX = 0x00,
    FIX_TYPE_DEAD_RECKONING = 0x01,
    FIX_TYPE_2D_FIX = 0x02,
    FIX_TYPE_3D_FIX = 0x03,
    FIX_TYPE_GNSS_DEAD_RECKONING = 0x04,
    FIX_TYPE_TIME_ONLY_FIX = 0x05
} gnss_fix_type_t;

/* UBX-NAV-PVT Message Structure (from datasheet) */
typedef struct {
    uint32_t iTOW;              /* GPS Millisecond Time of Week */
    uint16_t year;              /* Year (UTC) */
    uint8_t month;              /* Month (UTC) 1-12 */
    uint8_t day;                /* Day of Month (UTC) 1-31 */
    uint8_t hour;               /* Hour of Day (UTC) 0-23 */
    uint8_t min;                /* Minute of Hour (UTC) 0-59 */
    uint8_t sec;                /* Seconds of Minute (UTC) 0-60 */
    uint8_t valid;              /* Validity Flags */
    uint32_t tAcc;              /* Time Accuracy Estimate (ns) */
    int32_t nano;               /* Fraction of second (ns) */
    uint8_t fixType;            /* GNSS Fix Type */
    uint8_t flags;              /* Navigation Status Flags */
    uint8_t numSV;              /* Number of Satellites Used in Solution */
    int32_t lon;                /* Longitude (1e-7 degree) */
    int32_t lat;                /* Latitude (1e-7 degree) */
    int32_t height;             /* Height above Ellipsoid (mm) */
    int32_t hMSL;               /* Height above Mean Sea Level (mm) */
    uint32_t hAcc;              /* Horizontal Accuracy Estimate (mm) */
    uint32_t vAcc;              /* Vertical Accuracy Estimate (mm) */
    int32_t velN;               /* North Velocity (mm/s) */
    int32_t velE;               /* East Velocity (mm/s) */
    int32_t velD;               /* Down Velocity (mm/s) */
    int32_t gSpeed;             /* Ground Speed (mm/s) */
    int32_t heading;            /* Heading of Motion (1e-5 degree) */
    uint32_t sAcc;              /* Speed Accuracy Estimate (mm/s) */
    uint32_t headingAcc;        /* Heading Accuracy Estimate (1e-5 degree) */
    uint16_t pDOP;              /* Position DOP (1e-2) */
    int32_t headVeh;            /* Heading of Vehicle (1e-5 degree) */
} ubx_nav_pvt_t;

/* GNSS Driver Context Structure */
typedef struct {
    UART_HandleTypeDef *huart;
    gnss_state_t state;
    uint8_t rx_buffer[MAX_M10M_RX_BUFFER_SIZE];
    uint8_t tx_buffer[MAX_M10M_TX_BUFFER_SIZE];
    uint16_t rx_index;
    uint16_t rx_length;
    ubx_nav_pvt_t pvt_data;
    bool new_data_available;
    bool ack_received;
} gnss_driver_t;

/* Initialization and Control Functions */
void GNSS_Init(gnss_driver_t *driver, UART_HandleTypeDef *huart);
void GNSS_HardwareReset(void);
void GNSS_PowerOn(void);
void GNSS_PowerOff(void);
void GNSS_Process(gnss_driver_t *driver);

/* Configuration Functions */
void GNSS_SetMeasurementRate(gnss_driver_t *driver, uint16_t rate_ms);
void GNSS_RequestPVT(gnss_driver_t *driver);
void GNSS_EnableNMEA(gnss_driver_t *driver, uint8_t enable);

/* Data Retrieval Functions */
bool GNSS_GetPVT(gnss_driver_t *driver, ubx_nav_pvt_t *pvt);
gnss_fix_type_t GNSS_GetFixType(gnss_driver_t *driver);
uint8_t GNSS_GetSatelliteCount(gnss_driver_t *driver);
bool GNSS_GetPosition(gnss_driver_t *driver, double *lat, double *lon);
double GNSS_GetAltitude(gnss_driver_t *driver);
bool GNSS_IsFixValid(gnss_driver_t *driver);

/* UART Callback */
void GNSS_UART_RxCallback(gnss_driver_t *driver, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* MAX_M10M_H */
