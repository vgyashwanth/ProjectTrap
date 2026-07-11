/**
 ******************************************************************************
 * @file           : max_m10m_i2c.c
 * @brief          : MAX-M10M-00B GNSS Module I2C Driver Implementation
 * @attention
 * Driver for u-blox MAX-M10M-00B GNSS module via I2C interface
 * Communication: I2C1 @ 400 kHz (Fast-mode), 7-bit addressing
 * I2C Address: 0x42 (slave address)
 * Based on MAX-M10M-00B datasheet (UBX-22028884)
 ******************************************************************************
 */

#include "max_m10m.h"
#include <string.h>

/* Private function prototypes */
static bool GNSS_I2C_Read(gnss_i2c_driver_t *driver, uint8_t *buffer, uint16_t length);
static bool GNSS_I2C_Write(gnss_i2c_driver_t *driver, const uint8_t *data, uint16_t length);
static void GNSS_SendUBXMessage_I2C(gnss_i2c_driver_t *driver, uint8_t class, uint8_t id,
                                     const uint8_t *payload, uint16_t payload_len);
static uint8_t GNSS_CalculateChecksum_I2C(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b);
static bool GNSS_ParseUBXMessage_I2C(gnss_i2c_driver_t *driver, const uint8_t *buffer, uint16_t length);
static bool GNSS_ParseNAVPVT_I2C(gnss_i2c_driver_t *driver, const uint8_t *payload, uint16_t length);
static uint32_t GNSS_ReadU32_I2C(const uint8_t *buffer);
static int32_t GNSS_ReadI32_I2C(const uint8_t *buffer);
static uint16_t GNSS_ReadU16_I2C(const uint8_t *buffer);

/**
 * @brief Initialize GNSS driver for I2C interface
 * @param driver Pointer to GNSS I2C driver structure
 * @param hi2c I2C handle (I2C1)
 */
void GNSS_I2C_Init(gnss_i2c_driver_t *driver, I2C_HandleTypeDef *hi2c)
{
    if (driver == NULL || hi2c == NULL) {
        return;
    }

    /* Initialize driver structure */
    driver->hi2c = hi2c;
    driver->state = GNSS_I2C_STATE_INITIALIZING;
    driver->rx_index = 0;
    driver->rx_length = 0;
    driver->new_data_available = false;
    driver->ack_received = false;
    driver->msg_pending = false;

    /* Clear buffers */
    memset(driver->rx_buffer, 0, GNSS_RX_BUFFER_SIZE);
    memset(driver->tx_buffer, 0, GNSS_TX_BUFFER_SIZE);
    memset(&driver->pvt_data, 0, sizeof(ubx_nav_pvt_t));

    /* Power on GNSS module */
    GNSS_PowerOn();

    /* Perform hardware reset */
    GNSS_HardwareReset();

    /* Wait for device to be ready on I2C bus */
    HAL_Delay(100);

    /* Request PVT data */
    GNSS_I2C_RequestPVT(driver);

    driver->state = GNSS_I2C_STATE_READY;
}

/**
 * @brief Deinitialize GNSS driver
 */
void GNSS_I2C_Deinit(gnss_i2c_driver_t *driver)
{
    if (driver == NULL) {
        return;
    }

    GNSS_PowerOff();
    driver->state = GNSS_I2C_STATE_UNINITIALIZED;
}

/* Hardware control functions are implemented in max_m10m_uart.c as common functions */

/**
 * @brief Read data from I2C device
 * I2C address: 0x42 (7-bit), will be shifted to 0x84 (8-bit write) / 0x85 (8-bit read)
 */
static bool GNSS_I2C_Read(gnss_i2c_driver_t *driver, uint8_t *buffer, uint16_t length)
{
    HAL_StatusTypeDef status;

    if (driver == NULL || buffer == NULL || length == 0) {
        return false;
    }

    /* Read from I2C device at address 0x42 */
    status = HAL_I2C_Master_Receive(driver->hi2c, (MAX_M10M_I2C_ADDRESS << 1), buffer, length, I2C_TIMEOUT_MS);

    return (status == HAL_OK);
}

/**
 * @brief Write data to I2C device
 */
static bool GNSS_I2C_Write(gnss_i2c_driver_t *driver, const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    if (driver == NULL || data == NULL || length == 0) {
        return false;
    }

    /* Write to I2C device at address 0x42 */
    status = HAL_I2C_Master_Transmit(driver->hi2c, (MAX_M10M_I2C_ADDRESS << 1), (uint8_t *)data, length, I2C_TIMEOUT_MS);

    return (status == HAL_OK);
}

/**
 * @brief Send UBX message via I2C
 * Frame structure: [SYNC1][SYNC2][CLASS][ID][LEN_L][LEN_H][PAYLOAD][CK_A][CK_B]
 */
static void GNSS_SendUBXMessage_I2C(gnss_i2c_driver_t *driver, uint8_t class, uint8_t id,
                                     const uint8_t *payload, uint16_t payload_len)
{
    uint16_t idx = 0;
    uint8_t ck_a = 0, ck_b = 0;

    if (driver == NULL) {
        return;
    }

    /* Sync characters */
    driver->tx_buffer[idx++] = UBX_SYNC_CHAR_1;
    driver->tx_buffer[idx++] = UBX_SYNC_CHAR_2;

    /* Header */
    driver->tx_buffer[idx++] = class;
    driver->tx_buffer[idx++] = id;

    /* Length (little-endian) */
    driver->tx_buffer[idx++] = (payload_len & 0xFF);
    driver->tx_buffer[idx++] = ((payload_len >> 8) & 0xFF);

    /* Copy payload */
    if (payload != NULL && payload_len > 0) {
        memcpy(&driver->tx_buffer[idx], payload, payload_len);
        idx += payload_len;
    }

    /* Calculate checksum on class, id, length, and payload */
    GNSS_CalculateChecksum_I2C(&driver->tx_buffer[2],
                               2 + 2 + payload_len,  /* class + id + length fields + payload */
                               &ck_a, &ck_b);

    /* Add checksum */
    driver->tx_buffer[idx++] = ck_a;
    driver->tx_buffer[idx++] = ck_b;

    /* Send via I2C */
    GNSS_I2C_Write(driver, driver->tx_buffer, idx);
}

/**
 * @brief Calculate Fletcher checksum for UBX messages
 */
static uint8_t GNSS_CalculateChecksum_I2C(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0;
    *ck_b = 0;

    for (uint16_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }

    return 0;
}

/**
 * @brief Request NAV-PVT message from receiver
 */
void GNSS_I2C_RequestPVT(gnss_i2c_driver_t *driver)
{
    if (driver == NULL) {
        return;
    }

    /* Send UBX-NAV-PVT poll request (no payload) */
    GNSS_SendUBXMessage_I2C(driver, UBX_CLASS_NAV, UBX_NAV_PVT, NULL, 0);
    driver->msg_pending = true;
}

/**
 * @brief Set measurement rate (update rate)
 * @param rate_ms Measurement rate in milliseconds (e.g., 1000 for 1 Hz, 100 for 10 Hz)
 */
void GNSS_I2C_SetMeasurementRate(gnss_i2c_driver_t *driver, uint16_t rate_ms)
{
    uint8_t payload[6];

    if (driver == NULL) {
        return;
    }

    /* CFG-RATE payload: measRate(2), navRate(2), timeRef(2) */
    payload[0] = (rate_ms & 0xFF);
    payload[1] = ((rate_ms >> 8) & 0xFF);
    payload[2] = 1;          /* navRate = 1 (measurement cycle) */
    payload[3] = 0;
    payload[4] = 0;          /* timeRef = 0 (UTC) */
    payload[5] = 0;

    GNSS_SendUBXMessage_I2C(driver, UBX_CLASS_CFG, UBX_CFG_RATE, payload, 6);
}

/**
 * @brief Enable/Disable NMEA output
 * @param enable 1 to enable, 0 to disable
 */
void GNSS_I2C_EnableNMEA(gnss_i2c_driver_t *driver, uint8_t enable)
{
    uint8_t payload[3];

    if (driver == NULL) {
        return;
    }

    /* Enable all NMEA messages (GGA = 0x00) */
    payload[0] = 0x00;  /* Message ID (GGA) */
    payload[1] = enable ? 1 : 0;  /* Rate */
    payload[2] = 0;

    GNSS_SendUBXMessage_I2C(driver, UBX_CLASS_CFG, UBX_CFG_MSG, payload, 3);
}

/**
 * @brief Parse received UBX message
 */
static bool GNSS_ParseUBXMessage_I2C(gnss_i2c_driver_t *driver, const uint8_t *buffer, uint16_t length)
{
    uint8_t ck_a = 0, ck_b = 0;
    uint8_t class, id;
    uint16_t payload_len;

    if (length < 8) {  /* Minimum: SYNC(2) + CLASS + ID + LEN(2) + CK(2) */
        return false;
    }

    /* Verify sync characters */
    if (buffer[0] != UBX_SYNC_CHAR_1 || buffer[1] != UBX_SYNC_CHAR_2) {
        return false;
    }

    class = buffer[2];
    id = buffer[3];
    payload_len = (buffer[5] << 8) | buffer[4];

    /* Verify message length */
    if ((payload_len + 8) != length) {
        return false;
    }

    /* Verify checksum */
    GNSS_CalculateChecksum_I2C(&buffer[2], payload_len + 2, &ck_a, &ck_b);
    if (ck_a != buffer[length - 2] || ck_b != buffer[length - 1]) {
        return false;
    }

    /* Parse specific message types */
    if (class == UBX_CLASS_NAV && id == UBX_NAV_PVT) {
        return GNSS_ParseNAVPVT_I2C(driver, &buffer[6], payload_len);
    } else if (class == UBX_CLASS_ACK) {
        driver->ack_received = true;
        return true;
    }

    return false;
}

/**
 * @brief Parse NAV-PVT message
 */
static bool GNSS_ParseNAVPVT_I2C(gnss_i2c_driver_t *driver, const uint8_t *payload, uint16_t length)
{
    if (length < 92) {  /* Minimum NAV-PVT payload size */
        return false;
    }

    uint16_t idx = 0;

    /* Parse all fields according to datasheet */
    driver->pvt_data.iTOW = GNSS_ReadU32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.year = GNSS_ReadU16_I2C(&payload[idx]);
    idx += 2;

    driver->pvt_data.month = payload[idx++];
    driver->pvt_data.day = payload[idx++];
    driver->pvt_data.hour = payload[idx++];
    driver->pvt_data.min = payload[idx++];
    driver->pvt_data.sec = payload[idx++];

    driver->pvt_data.valid = payload[idx++];

    driver->pvt_data.tAcc = GNSS_ReadU32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.nano = GNSS_ReadI32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.fixType = payload[idx++];
    driver->pvt_data.flags = payload[idx++];
    driver->pvt_data.numSV = payload[idx++];

    idx += 1;  /* Reserved byte */

    driver->pvt_data.lon = GNSS_ReadI32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.lat = GNSS_ReadI32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.height = GNSS_ReadI32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.hMSL = GNSS_ReadI32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.hAcc = GNSS_ReadU32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.vAcc = GNSS_ReadU32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.velN = GNSS_ReadI32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.velE = GNSS_ReadI32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.velD = GNSS_ReadI32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.gSpeed = GNSS_ReadI32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.heading = GNSS_ReadI32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.sAcc = GNSS_ReadU32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.headingAcc = GNSS_ReadU32_I2C(&payload[idx]);
    idx += 4;

    driver->pvt_data.pDOP = GNSS_ReadU16_I2C(&payload[idx]);
    idx += 2;

    idx += 6;  /* Reserved bytes */

    driver->pvt_data.headVeh = GNSS_ReadI32_I2C(&payload[idx]);
    idx += 4;

    driver->new_data_available = true;
    driver->msg_pending = false;

    if (driver->pvt_data.fixType >= FIX_TYPE_2D_FIX) {
        driver->state = GNSS_I2C_STATE_FIX_ACQUIRED;
    } else {
        driver->state = GNSS_I2C_STATE_NO_FIX;
    }

    return true;
}

/**
 * @brief Poll and process GNSS data via I2C
 * Call this periodically from main loop or timer
 */
void GNSS_I2C_Process(gnss_i2c_driver_t *driver)
{
    uint16_t bytes_available = 0;

    if (driver == NULL || driver->hi2c == NULL) {
        return;
    }

    /* Try to read data from I2C device */
    /* First, read available byte count (I2C protocol: first 2 bytes indicate payload length) */
    uint8_t length_bytes[2];
    if (!GNSS_I2C_Read(driver, length_bytes, 2)) {
        return;
    }

    bytes_available = (length_bytes[0]) | (length_bytes[1] << 8);

    /* Verify reasonable length */
    if (bytes_available == 0 || bytes_available > GNSS_RX_BUFFER_SIZE) {
        return;
    }

    /* Read the actual message */
    if (!GNSS_I2C_Read(driver, driver->rx_buffer, bytes_available)) {
        return;
    }

    /* Parse the received message */
    GNSS_ParseUBXMessage_I2C(driver, driver->rx_buffer, bytes_available);

    /* If more data is pending, request next message */
    if (!driver->new_data_available) {
        GNSS_I2C_RequestPVT(driver);
    }
}

/**
 * @brief Get parsed PVT data
 */
bool GNSS_I2C_GetPVT(gnss_i2c_driver_t *driver, ubx_nav_pvt_t *pvt)
{
    if (driver == NULL || pvt == NULL || !driver->new_data_available) {
        return false;
    }

    memcpy(pvt, &driver->pvt_data, sizeof(ubx_nav_pvt_t));
    driver->new_data_available = false;
    return true;
}

/**
 * @brief Get current fix type
 */
gnss_fix_type_t GNSS_I2C_GetFixType(gnss_i2c_driver_t *driver)
{
    if (driver == NULL) {
        return FIX_TYPE_NO_FIX;
    }

    return (gnss_fix_type_t)driver->pvt_data.fixType;
}

/**
 * @brief Get number of satellites used
 */
uint8_t GNSS_I2C_GetSatelliteCount(gnss_i2c_driver_t *driver)
{
    if (driver == NULL) {
        return 0;
    }

    return driver->pvt_data.numSV;
}

/**
 * @brief Get position (latitude and longitude in degrees)
 * Internal storage uses 1e-7 degree format
 */
bool GNSS_I2C_GetPosition(gnss_i2c_driver_t *driver, double *lat, double *lon)
{
    if (driver == NULL || lat == NULL || lon == NULL) {
        return false;
    }

    if (!GNSS_I2C_IsFixValid(driver)) {
        return false;
    }

    *lat = driver->pvt_data.lat / 1e7;
    *lon = driver->pvt_data.lon / 1e7;

    return true;
}

/**
 * @brief Get altitude in meters
 * Internal storage uses mm, so divide by 1000
 */
double GNSS_I2C_GetAltitude(gnss_i2c_driver_t *driver)
{
    if (driver == NULL) {
        return 0.0;
    }

    return driver->pvt_data.height / 1000.0;
}

/**
 * @brief Get ground speed in m/s
 */
double GNSS_I2C_GetSpeed(gnss_i2c_driver_t *driver)
{
    if (driver == NULL) {
        return 0.0;
    }

    return driver->pvt_data.gSpeed / 1000.0;
}

/**
 * @brief Get heading in degrees
 */
double GNSS_I2C_GetHeading(gnss_i2c_driver_t *driver)
{
    if (driver == NULL) {
        return 0.0;
    }

    return driver->pvt_data.heading / 1e5;
}

/**
 * @brief Check if position fix is valid
 */
bool GNSS_I2C_IsFixValid(gnss_i2c_driver_t *driver)
{
    if (driver == NULL) {
        return false;
    }

    return (driver->pvt_data.fixType >= FIX_TYPE_2D_FIX);
}

/**
 * @brief Get current driver state
 */
gnss_i2c_state_t GNSS_I2C_GetState(gnss_i2c_driver_t *driver)
{
    if (driver == NULL) {
        return GNSS_I2C_STATE_UNINITIALIZED;
    }

    return driver->state;
}

/* Helper functions for little-endian data reading */
static uint32_t GNSS_ReadU32_I2C(const uint8_t *buffer)
{
    return (buffer[0]) | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
}

static int32_t GNSS_ReadI32_I2C(const uint8_t *buffer)
{
    return (int32_t)GNSS_ReadU32_I2C(buffer);
}

static uint16_t GNSS_ReadU16_I2C(const uint8_t *buffer)
{
    return (buffer[0]) | (buffer[1] << 8);
}

