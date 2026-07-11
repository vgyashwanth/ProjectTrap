/**
 ******************************************************************************
 * @file           : max_m10m.h
 * @brief          : MAX-M10M-00B GNSS Module Driver Header (UART & I2C)
 * @attention
 * Unified driver for u-blox MAX-M10M-00B GNSS module
 * Supports both UART (9600 baud) and I2C (400 kHz) interfaces
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

/* ============================================================================
 * COMMON UBX PROTOCOL DEFINITIONS (Interface Independent)
 * ============================================================================ */

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
#define GNSS_RX_BUFFER_SIZE         256
#define GNSS_TX_BUFFER_SIZE         128

/* ============================================================================
 * COMMON DATA STRUCTURES
 * ============================================================================ */

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

/* ============================================================================
 * UART INTERFACE DEFINITIONS
 * ============================================================================ */

typedef enum {
    GNSS_STATE_UNINITIALIZED = 0,
    GNSS_STATE_INITIALIZING,
    GNSS_STATE_READY,
    GNSS_STATE_NO_FIX,
    GNSS_STATE_FIX_ACQUIRED,
    GNSS_STATE_ERROR
} gnss_uart_state_t;

/* GNSS UART Driver Context Structure */
typedef struct {
    UART_HandleTypeDef *huart;
    gnss_uart_state_t state;
    uint8_t rx_buffer[GNSS_RX_BUFFER_SIZE];
    uint8_t tx_buffer[GNSS_TX_BUFFER_SIZE];
    uint16_t rx_index;
    uint16_t rx_length;
    ubx_nav_pvt_t pvt_data;
    bool new_data_available;
    bool ack_received;
} gnss_uart_driver_t;

/* UART Interface Functions */
void GNSS_UART_Init(gnss_uart_driver_t *driver, UART_HandleTypeDef *huart);
void GNSS_UART_Deinit(gnss_uart_driver_t *driver);
void GNSS_UART_Process(gnss_uart_driver_t *driver);
void GNSS_UART_RxCallback(gnss_uart_driver_t *driver, uint8_t data);
void GNSS_UART_SetMeasurementRate(gnss_uart_driver_t *driver, uint16_t rate_ms);
void GNSS_UART_RequestPVT(gnss_uart_driver_t *driver);
void GNSS_UART_EnableNMEA(gnss_uart_driver_t *driver, uint8_t enable);

/* ============================================================================
 * I2C INTERFACE DEFINITIONS
 * ============================================================================ */

#define MAX_M10M_I2C_ADDRESS        0x42    /* Slave address (7-bit) */
#define I2C_TIMEOUT_MS              1000

typedef enum {
    GNSS_I2C_STATE_UNINITIALIZED = 0,
    GNSS_I2C_STATE_INITIALIZING,
    GNSS_I2C_STATE_READY,
    GNSS_I2C_STATE_NO_FIX,
    GNSS_I2C_STATE_FIX_ACQUIRED,
    GNSS_I2C_STATE_ERROR
} gnss_i2c_state_t;

/* GNSS I2C Driver Context Structure */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    gnss_i2c_state_t state;
    uint8_t rx_buffer[GNSS_RX_BUFFER_SIZE];
    uint8_t tx_buffer[GNSS_TX_BUFFER_SIZE];
    uint16_t rx_index;
    uint16_t rx_length;
    ubx_nav_pvt_t pvt_data;
    bool new_data_available;
    bool ack_received;
    bool msg_pending;
} gnss_i2c_driver_t;

/* I2C Interface Functions */
void GNSS_I2C_Init(gnss_i2c_driver_t *driver, I2C_HandleTypeDef *hi2c);
void GNSS_I2C_Deinit(gnss_i2c_driver_t *driver);
void GNSS_I2C_Process(gnss_i2c_driver_t *driver);
void GNSS_I2C_SetMeasurementRate(gnss_i2c_driver_t *driver, uint16_t rate_ms);
void GNSS_I2C_RequestPVT(gnss_i2c_driver_t *driver);
void GNSS_I2C_EnableNMEA(gnss_i2c_driver_t *driver, uint8_t enable);

/* ============================================================================
 * COMMON INTERFACE FUNCTIONS (Hardware Control)
 * ============================================================================ */

/* Hardware Reset and Power Control (Same for both UART and I2C) */
void GNSS_HardwareReset(void);
void GNSS_PowerOn(void);
void GNSS_PowerOff(void);

/* ============================================================================
 * COMMON DATA RETRIEVAL FUNCTIONS (Interface Independent)
 * ============================================================================ */

/* UART Interface Data Getters */
bool GNSS_UART_GetPVT(gnss_uart_driver_t *driver, ubx_nav_pvt_t *pvt);
gnss_fix_type_t GNSS_UART_GetFixType(gnss_uart_driver_t *driver);
uint8_t GNSS_UART_GetSatelliteCount(gnss_uart_driver_t *driver);
bool GNSS_UART_GetPosition(gnss_uart_driver_t *driver, double *lat, double *lon);
double GNSS_UART_GetAltitude(gnss_uart_driver_t *driver);
double GNSS_UART_GetSpeed(gnss_uart_driver_t *driver);
double GNSS_UART_GetHeading(gnss_uart_driver_t *driver);
bool GNSS_UART_IsFixValid(gnss_uart_driver_t *driver);

/* I2C Interface Data Getters */
bool GNSS_I2C_GetPVT(gnss_i2c_driver_t *driver, ubx_nav_pvt_t *pvt);
gnss_fix_type_t GNSS_I2C_GetFixType(gnss_i2c_driver_t *driver);
uint8_t GNSS_I2C_GetSatelliteCount(gnss_i2c_driver_t *driver);
bool GNSS_I2C_GetPosition(gnss_i2c_driver_t *driver, double *lat, double *lon);
double GNSS_I2C_GetAltitude(gnss_i2c_driver_t *driver);
double GNSS_I2C_GetSpeed(gnss_i2c_driver_t *driver);
double GNSS_I2C_GetHeading(gnss_i2c_driver_t *driver);
bool GNSS_I2C_IsFixValid(gnss_i2c_driver_t *driver);
gnss_i2c_state_t GNSS_I2C_GetState(gnss_i2c_driver_t *driver);

#ifdef __cplusplus
}
#endif

#endif /* MAX_M10M_H */
