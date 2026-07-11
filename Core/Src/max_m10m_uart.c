/**
 ******************************************************************************
 * @file           : max_m10m_uart.c
 * @brief          : MAX-M10M-00B GNSS Module UART Driver Implementation
 * @attention
 * Driver for u-blox MAX-M10M-00B GNSS module via UART interface
 * Communication: UART2 @ 9600 baud, 8 data bits, 1 stop bit, no parity
 * Based on MAX-M10M-00B datasheet (UBX-22028884)
 ******************************************************************************
 */

#include "max_m10m.h"
#include <string.h>

/* Private function prototypes */
static void GNSS_UART_SendUBXMessage(gnss_uart_driver_t *driver, uint8_t class, uint8_t id,
                                      const uint8_t *payload, uint16_t payload_len);
static uint8_t GNSS_UART_CalculateChecksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b);
static bool GNSS_UART_ParseUBXMessage(gnss_uart_driver_t *driver, const uint8_t *buffer, uint16_t length);
static bool GNSS_UART_ParseNAVPVT(gnss_uart_driver_t *driver, const uint8_t *payload, uint16_t length);
static uint32_t GNSS_UART_ReadU32(const uint8_t *buffer);
static int32_t GNSS_UART_ReadI32(const uint8_t *buffer);
static uint16_t GNSS_UART_ReadU16(const uint8_t *buffer);

/**
 * @brief Initialize GNSS driver and hardware (UART interface)
 * @param driver Pointer to GNSS driver structure
 * @param huart UART handle
 */
void GNSS_UART_Init(gnss_uart_driver_t *driver, UART_HandleTypeDef *huart)
{
    if (driver == NULL || huart == NULL) {
        return;
    }

    /* Initialize driver structure */
    driver->huart = huart;
    driver->state = GNSS_STATE_INITIALIZING;
    driver->rx_index = 0;
    driver->rx_length = 0;
    driver->new_data_available = false;
    driver->ack_received = false;

    /* Clear buffers */
    memset(driver->rx_buffer, 0, GNSS_RX_BUFFER_SIZE);
    memset(driver->tx_buffer, 0, GNSS_TX_BUFFER_SIZE);
    memset(&driver->pvt_data, 0, sizeof(ubx_nav_pvt_t));

    /* Power on GNSS module */
    GNSS_PowerOn();

    /* Perform hardware reset */
    GNSS_HardwareReset();

    /* Request PVT data */
    GNSS_UART_RequestPVT(driver);

    driver->state = GNSS_STATE_READY;
}

/**
 * @brief Hardware reset of GNSS module
 * Reset pin (PA6 - GNSS_RST) is active low, requires min 1ms pulse
 */
void GNSS_HardwareReset(void)
{
    /* Pull reset pin low */
    HAL_GPIO_WritePin(GNSS_RST_GPIO_Port, GNSS_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);  /* At least 1ms */

    /* Release reset pin */
    HAL_GPIO_WritePin(GNSS_RST_GPIO_Port, GNSS_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(500);  /* Wait for boot sequence */
}

/**
 * @brief Power on GNSS module
 * Enable pin (PA7 - GNSS_EN) controls the receiver enable
 */
void GNSS_PowerOn(void)
{
    HAL_GPIO_WritePin(GNSS_EN_GPIO_Port, GNSS_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
}

/**
 * @brief Power off GNSS module
 */
void GNSS_PowerOff(void)
{
    HAL_GPIO_WritePin(GNSS_EN_GPIO_Port, GNSS_EN_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Send UBX message via UART
 * Frame structure: [SYNC1][SYNC2][CLASS][ID][LEN_L][LEN_H][PAYLOAD][CK_A][CK_B]
 */
static void GNSS_UART_SendUBXMessage(gnss_uart_driver_t *driver, uint8_t class, uint8_t id,
                                      const uint8_t *payload, uint16_t payload_len)
{
    uint16_t idx = 0;
    uint8_t ck_a = 0, ck_b = 0;

    if (driver == NULL || driver->huart == NULL) {
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
    GNSS_UART_CalculateChecksum(&driver->tx_buffer[2],
                                2 + 2 + payload_len,  /* class + id + length fields + payload */
                                &ck_a, &ck_b);

    /* Add checksum */
    driver->tx_buffer[idx++] = ck_a;
    driver->tx_buffer[idx++] = ck_b;

    /* Send via UART */
    HAL_UART_Transmit(driver->huart, driver->tx_buffer, idx, 1000);
}

/**
 * @brief Calculate Fletcher checksum for UBX messages
 */
static uint8_t GNSS_UART_CalculateChecksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b)
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
void GNSS_UART_RequestPVT(gnss_uart_driver_t *driver)
{
    if (driver == NULL) {
        return;
    }

    /* Send UBX-NAV-PVT poll request (no payload) */
    GNSS_UART_SendUBXMessage(driver, UBX_CLASS_NAV, UBX_NAV_PVT, NULL, 0);
}

/**
 * @brief Set measurement rate (update rate)
 * @param rate_ms Measurement rate in milliseconds (e.g., 1000 for 1 Hz, 100 for 10 Hz)
 */
void GNSS_UART_SetMeasurementRate(gnss_uart_driver_t *driver, uint16_t rate_ms)
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

    GNSS_UART_SendUBXMessage(driver, UBX_CLASS_CFG, UBX_CFG_RATE, payload, 6);
}

/**
 * @brief Enable/Disable NMEA output
 * @param enable 1 to enable, 0 to disable
 */
void GNSS_UART_EnableNMEA(gnss_uart_driver_t *driver, uint8_t enable)
{
    uint8_t payload[3];

    if (driver == NULL) {
        return;
    }

    /* Enable all NMEA messages (GGA, GLL, GSA, GSV, RMC, VTG) */
    payload[0] = 0x00;  /* Message ID (GGA = 0x00) */
    payload[1] = enable ? 1 : 0;  /* Rate */
    payload[2] = 0;

    /* Send for each message type */
    GNSS_UART_SendUBXMessage(driver, UBX_CLASS_CFG, UBX_CFG_MSG, payload, 3);
}

/**
 * @brief Parse received UBX message
 */
static bool GNSS_UART_ParseUBXMessage(gnss_uart_driver_t *driver, const uint8_t *buffer, uint16_t length)
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
    GNSS_UART_CalculateChecksum(&buffer[2], payload_len + 2, &ck_a, &ck_b);
    if (ck_a != buffer[length - 2] || ck_b != buffer[length - 1]) {
        return false;
    }

    /* Parse specific message types */
    if (class == UBX_CLASS_NAV && id == UBX_NAV_PVT) {
        return GNSS_UART_ParseNAVPVT(driver, &buffer[6], payload_len);
    } else if (class == UBX_CLASS_ACK) {
        driver->ack_received = true;
        return true;
    }

    return false;
}

/**
 * @brief Parse NAV-PVT message
 */
static bool GNSS_UART_ParseNAVPVT(gnss_uart_driver_t *driver, const uint8_t *payload, uint16_t length)
{
    if (length < 92) {  /* Minimum NAV-PVT payload size */
        return false;
    }

    uint16_t idx = 0;

    /* Parse all fields according to datasheet */
    driver->pvt_data.iTOW = GNSS_UART_ReadU32(&payload[idx]);
    idx += 4;

    driver->pvt_data.year = GNSS_UART_ReadU16(&payload[idx]);
    idx += 2;

    driver->pvt_data.month = payload[idx++];
    driver->pvt_data.day = payload[idx++];
    driver->pvt_data.hour = payload[idx++];
    driver->pvt_data.min = payload[idx++];
    driver->pvt_data.sec = payload[idx++];

    driver->pvt_data.valid = payload[idx++];

    driver->pvt_data.tAcc = GNSS_UART_ReadU32(&payload[idx]);
    idx += 4;

    driver->pvt_data.nano = GNSS_UART_ReadI32(&payload[idx]);
    idx += 4;

    driver->pvt_data.fixType = payload[idx++];
    driver->pvt_data.flags = payload[idx++];
    driver->pvt_data.numSV = payload[idx++];

    idx += 1;  /* Reserved byte */

    driver->pvt_data.lon = GNSS_UART_ReadI32(&payload[idx]);
    idx += 4;

    driver->pvt_data.lat = GNSS_UART_ReadI32(&payload[idx]);
    idx += 4;

    driver->pvt_data.height = GNSS_UART_ReadI32(&payload[idx]);
    idx += 4;

    driver->pvt_data.hMSL = GNSS_UART_ReadI32(&payload[idx]);
    idx += 4;

    driver->pvt_data.hAcc = GNSS_UART_ReadU32(&payload[idx]);
    idx += 4;

    driver->pvt_data.vAcc = GNSS_UART_ReadU32(&payload[idx]);
    idx += 4;

    driver->pvt_data.velN = GNSS_UART_ReadI32(&payload[idx]);
    idx += 4;

    driver->pvt_data.velE = GNSS_UART_ReadI32(&payload[idx]);
    idx += 4;

    driver->pvt_data.velD = GNSS_UART_ReadI32(&payload[idx]);
    idx += 4;

    driver->pvt_data.gSpeed = GNSS_UART_ReadI32(&payload[idx]);
    idx += 4;

    driver->pvt_data.heading = GNSS_UART_ReadI32(&payload[idx]);
    idx += 4;

    driver->pvt_data.sAcc = GNSS_UART_ReadU32(&payload[idx]);
    idx += 4;

    driver->pvt_data.headingAcc = GNSS_UART_ReadU32(&payload[idx]);
    idx += 4;

    driver->pvt_data.pDOP = GNSS_UART_ReadU16(&payload[idx]);
    idx += 2;

    idx += 6;  /* Reserved bytes */

    driver->pvt_data.headVeh = GNSS_UART_ReadI32(&payload[idx]);
    idx += 4;

    driver->new_data_available = true;

    if (driver->pvt_data.fixType >= FIX_TYPE_2D_FIX) {
        driver->state = GNSS_STATE_FIX_ACQUIRED;
    } else {
        driver->state = GNSS_STATE_NO_FIX;
    }

    return true;
}

/**
 * @brief UART receive callback for character-by-character reception
 */
void GNSS_UART_RxCallback(gnss_uart_driver_t *driver, uint8_t data)
{
    if (driver == NULL || driver->rx_index >= GNSS_RX_BUFFER_SIZE) {
        driver->rx_index = 0;
        return;
    }

    driver->rx_buffer[driver->rx_index] = data;
    driver->rx_index++;

    /* Try to detect complete message when we have sync bytes */
    if (driver->rx_index >= 2) {
        if (driver->rx_buffer[0] == UBX_SYNC_CHAR_1 &&
            driver->rx_buffer[1] == UBX_SYNC_CHAR_2) {

            /* Extract length field if we have it */
            if (driver->rx_index >= 6) {
                uint16_t msg_len = 8 + ((driver->rx_buffer[5] << 8) | driver->rx_buffer[4]);

                if (driver->rx_index == msg_len) {
                    /* Complete message received */
                    driver->rx_length = driver->rx_index;
                    driver->rx_index = 0;
                }
            }
        }
    }
}

/**
 * @brief Process received data and parse messages
 */
void GNSS_UART_Process(gnss_uart_driver_t *driver)
{
    if (driver == NULL || driver->rx_length == 0) {
        return;
    }

    /* Parse the received message */
    GNSS_UART_ParseUBXMessage(driver, driver->rx_buffer, driver->rx_length);

    /* Clear length to indicate message is processed */
    driver->rx_length = 0;
    driver->rx_index = 0;
}

/**
 * @brief Get parsed PVT data
 */
bool GNSS_UART_GetPVT(gnss_uart_driver_t *driver, ubx_nav_pvt_t *pvt)
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
gnss_fix_type_t GNSS_UART_GetFixType(gnss_uart_driver_t *driver)
{
    if (driver == NULL) {
        return FIX_TYPE_NO_FIX;
    }

    return (gnss_fix_type_t)driver->pvt_data.fixType;
}

/**
 * @brief Get number of satellites used
 */
uint8_t GNSS_UART_GetSatelliteCount(gnss_uart_driver_t *driver)
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
bool GNSS_UART_GetPosition(gnss_uart_driver_t *driver, double *lat, double *lon)
{
    if (driver == NULL || lat == NULL || lon == NULL) {
        return false;
    }

    if (!GNSS_UART_IsFixValid(driver)) {
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
double GNSS_UART_GetAltitude(gnss_uart_driver_t *driver)
{
    if (driver == NULL) {
        return 0.0;
    }

    return driver->pvt_data.height / 1000.0;
}

/**
 * @brief Get ground speed in m/s
 */
double GNSS_UART_GetSpeed(gnss_uart_driver_t *driver)
{
    if (driver == NULL) {
        return 0.0;
    }

    return driver->pvt_data.gSpeed / 1000.0;
}

/**
 * @brief Get heading in degrees
 */
double GNSS_UART_GetHeading(gnss_uart_driver_t *driver)
{
    if (driver == NULL) {
        return 0.0;
    }

    return driver->pvt_data.heading / 1e5;
}

/**
 * @brief Check if position fix is valid
 */
bool GNSS_UART_IsFixValid(gnss_uart_driver_t *driver)
{
    if (driver == NULL) {
        return false;
    }

    return (driver->pvt_data.fixType >= FIX_TYPE_2D_FIX);
}

/* Helper functions for little-endian data reading */
static uint32_t GNSS_UART_ReadU32(const uint8_t *buffer)
{
    return (buffer[0]) | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
}

static int32_t GNSS_UART_ReadI32(const uint8_t *buffer)
{
    return (int32_t)GNSS_UART_ReadU32(buffer);
}

static uint16_t GNSS_UART_ReadU16(const uint8_t *buffer)
{
    return (buffer[0]) | (buffer[1] << 8);
}

