/**
 ******************************************************************************
 * @file           : eg800q_uart.c
 * @brief          : Quectel EG800Q Series LTE Module UART Driver Implementation
 * @attention
 * Driver for Quectel EG800Q-CN LTE module via UART interface
 * Communication: USART1 @ 115200 baud, 8 data bits, 1 stop bit, no parity
 * Level Shifter Required: Module 1.8V <-> STM32C051 3.3V
 * Based on Quectel EG800Q-Series Hardware Design v1.4
 ******************************************************************************
 */

#include "eg800q.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private function prototypes */
static bool LTE_UART_ParseResponse(lte_uart_driver_t *driver, const uint8_t *buffer, uint16_t length);
static bool LTE_UART_CheckResponse(const char *response, const char *expected);
static char* LTE_UART_ExtractParameter(const char *response, const char *prefix, char *output, uint16_t max_len);
static uint32_t LTE_GetTicks(void);

/**
 * @brief Initialize LTE driver and hardware (UART interface)
 * @param driver Pointer to LTE driver structure
 * @param huart UART handle (USART1)
 */
void LTE_UART_Init(lte_uart_driver_t *driver, UART_HandleTypeDef *huart)
{
    if (driver == NULL || huart == NULL) {
        return;
    }

    /* Initialize driver structure */
    driver->huart = huart;
    driver->state = LTE_STATE_INITIALIZING;
    driver->rx_index = 0;
    driver->rx_length = 0;
    driver->new_response_available = false;
    driver->command_in_progress = false;
    driver->last_command_time = 0;

    /* Clear buffers */
    memset(driver->rx_buffer, 0, LTE_RX_BUFFER_SIZE);
    memset(driver->tx_buffer, 0, LTE_TX_BUFFER_SIZE);
    memset(driver->response_buffer, 0, LTE_RX_BUFFER_SIZE);
    memset(&driver->sim_info, 0, sizeof(lte_sim_info_t));
    memset(&driver->network_info, 0, sizeof(lte_network_info_t));
    memset(&driver->signal_quality, 0, sizeof(lte_signal_quality_t));

    /* Power on LTE module */
    LTE_PowerOn();

    /* Perform hardware reset */
    LTE_HardwareReset();

    driver->state = LTE_STATE_READY;
}

/**
 * @brief Deinitialize LTE driver
 * @param driver Pointer to LTE driver structure
 */
void LTE_UART_Deinit(lte_uart_driver_t *driver)
{
    if (driver == NULL) {
        return;
    }

    /* Power off module */
    LTE_PowerOff();

    driver->state = LTE_STATE_UNINITIALIZED;
}

/**
 * @brief Power on LTE module
 * Pull PWRKEY (PB1) low for minimum 500ms to power on
 */
void LTE_PowerOn(void)
{
    /* Pull PWRKEY low */
    HAL_GPIO_WritePin(LTE_PWR_KEY_GPIO_Port, LTE_PWR_KEY_Pin, GPIO_PIN_RESET);
    HAL_Delay(600);  /* 500ms minimum */

    /* Release PWRKEY */
    HAL_GPIO_WritePin(LTE_PWR_KEY_GPIO_Port, LTE_PWR_KEY_Pin, GPIO_PIN_SET);

    /* Wait for module to boot (~10s) */
    HAL_Delay(2000);
}

/**
 * @brief Power off LTE module
 * Pull PWRKEY low for minimum 650ms to power off
 */
void LTE_PowerOff(void)
{
    /* Pull PWRKEY low */
    HAL_GPIO_WritePin(LTE_PWR_KEY_GPIO_Port, LTE_PWR_KEY_Pin, GPIO_PIN_RESET);
    HAL_Delay(700);  /* 650ms minimum */

    /* Release PWRKEY */
    HAL_GPIO_WritePin(LTE_PWR_KEY_GPIO_Port, LTE_PWR_KEY_Pin, GPIO_PIN_SET);

    /* Wait for module to shutdown */
    HAL_Delay(2000);
}

/**
 * @brief Hardware reset of LTE module
 * Pull LTE_RST (PA15) low for minimum 300ms to reset
 */
void LTE_HardwareReset(void)
{
    /* Pull reset pin low */
    HAL_GPIO_WritePin(LTE_RST_GPIO_Port, LTE_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(300);  /* At least 300ms */

    /* Release reset pin */
    HAL_GPIO_WritePin(LTE_RST_GPIO_Port, LTE_RST_Pin, GPIO_PIN_SET);

    /* Wait for boot sequence */
    HAL_Delay(2000);
}

/**
 * @brief Set RTS/CTS flow control
 * @param enable 1 to enable, 0 to disable
 */
void LTE_SetFlowControl(uint8_t enable)
{
    if (enable) {
        /* Enable flow control - configure MCU UART to handle RTS/CTS */
        /* This is typically done in UART initialization, but can be modified here */
    } else {
        /* Disable flow control */
    }
}

/**
 * @brief Send AT command via UART
 * @param driver Pointer to LTE driver structure
 * @param command Command string (without terminator)
 */
void LTE_UART_SendCommand(lte_uart_driver_t *driver, const char *command)
{
    uint16_t cmd_len;

    if (driver == NULL || command == NULL || driver->huart == NULL) {
        return;
    }

    /* Clear response flags */
    driver->new_response_available = false;
    driver->command_in_progress = true;
    driver->last_command_time = LTE_GetTicks();

    /* Build command with terminator */
    cmd_len = strlen(command);
    if (cmd_len >= (LTE_TX_BUFFER_SIZE - 2)) {
        cmd_len = LTE_TX_BUFFER_SIZE - 3;
    }

    memcpy(driver->tx_buffer, command, cmd_len);
    driver->tx_buffer[cmd_len] = '\r';
    driver->tx_buffer[cmd_len + 1] = '\n';
    cmd_len += 2;

    /* Send via UART */
    HAL_UART_Transmit(driver->huart, driver->tx_buffer, cmd_len, 1000);
}

/**
 * @brief UART receive callback for character-by-character reception
 */
void LTE_UART_RxCallback(lte_uart_driver_t *driver, uint8_t data)
{
    if (driver == NULL || driver->rx_index >= LTE_RX_BUFFER_SIZE) {
        driver->rx_index = 0;
        return;
    }

    driver->rx_buffer[driver->rx_index] = data;
    driver->rx_index++;

    /* Check for end of response (CR+LF) */
    if (driver->rx_index >= 2) {
        if (driver->rx_buffer[driver->rx_index - 2] == '\r' &&
            driver->rx_buffer[driver->rx_index - 1] == '\n') {

            /* Look for final response (OK, ERROR, etc.) */
            if (strstr((char*)driver->rx_buffer, "OK") != NULL ||
                strstr((char*)driver->rx_buffer, "ERROR") != NULL ||
                strstr((char*)driver->rx_buffer, "CONNECT") != NULL) {

                driver->rx_length = driver->rx_index;
                driver->rx_index = 0;
                driver->new_response_available = true;
                driver->command_in_progress = false;
            }
        }
    }
}

/**
 * @brief Process received data and parse responses
 */
void LTE_UART_Process(lte_uart_driver_t *driver)
{
    if (driver == NULL || driver->rx_length == 0) {
        return;
    }

    /* Copy to response buffer */
    if (driver->rx_length <= LTE_RX_BUFFER_SIZE) {
        memcpy(driver->response_buffer, driver->rx_buffer, driver->rx_length);
        driver->response_length = driver->rx_length;
    }

    /* Parse the received message */
    LTE_UART_ParseResponse(driver, driver->rx_buffer, driver->rx_length);

    /* Clear length to indicate message is processed */
    driver->rx_length = 0;
    driver->rx_index = 0;
}

/**
 * @brief Wait for response from module
 * @param driver Pointer to LTE driver structure
 * @param timeout_ms Timeout in milliseconds
 * @return true if response received, false if timeout
 */
bool LTE_UART_WaitResponse(lte_uart_driver_t *driver, uint32_t timeout_ms)
{
    uint32_t start_time;

    if (driver == NULL) {
        return false;
    }

    start_time = LTE_GetTicks();

    while (!driver->new_response_available) {
        if ((LTE_GetTicks() - start_time) > timeout_ms) {
            return false;
        }
        HAL_Delay(10);  /* Small delay to prevent busy waiting */
    }

    return true;
}

/**
 * @brief Get response buffer content
 * @param driver Pointer to LTE driver structure
 * @param response Output buffer for response string
 * @param max_len Maximum length to copy
 * @return true if response available, false otherwise
 */
bool LTE_UART_GetResponse(lte_uart_driver_t *driver, char *response, uint16_t max_len)
{
    if (driver == NULL || response == NULL || !driver->new_response_available) {
        return false;
    }

    uint16_t copy_len = (driver->response_length < max_len) ? driver->response_length : (max_len - 1);
    memcpy(response, driver->response_buffer, copy_len);
    response[copy_len] = '\0';

    driver->new_response_available = false;

    return true;
}

/**
 * @brief Parse response message
 */
static bool LTE_UART_ParseResponse(lte_uart_driver_t *driver, const uint8_t *buffer, uint16_t length)
{
    if (buffer == NULL || length == 0) {
        return false;
    }

    /* Check for OK response */
    if (strstr((char*)buffer, "OK") != NULL) {
        return true;
    }

    /* Check for ERROR response */
    if (strstr((char*)buffer, "ERROR") != NULL) {
        return false;
    }

    return false;
}

/**
 * @brief Check if response contains expected string
 */
static bool LTE_UART_CheckResponse(const char *response, const char *expected)
{
    if (response == NULL || expected == NULL) {
        return false;
    }

    return (strstr(response, expected) != NULL);
}

/**
 * @brief Extract parameter from response
 * Example: response="+QMTCONN: 0,0" prefix="+QMTCONN: " -> output="0,0"
 */
static char* LTE_UART_ExtractParameter(const char *response, const char *prefix, char *output, uint16_t max_len)
{
    const char *start;
    uint16_t len;

    if (response == NULL || prefix == NULL || output == NULL) {
        return NULL;
    }

    start = strstr(response, prefix);
    if (start == NULL) {
        return NULL;
    }

    start += strlen(prefix);

    /* Copy until CR/LF or end */
    len = 0;
    while (len < (max_len - 1) && start[len] != '\r' && start[len] != '\n' && start[len] != '\0') {
        output[len] = start[len];
        len++;
    }

    output[len] = '\0';

    return output;
}

/**
 * @brief Startup sequence for LTE module
 */
bool LTE_Startup(lte_uart_driver_t *driver)
{
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL) {
        return false;
    }

    /* Send AT to verify communication */
    LTE_UART_SendCommand(driver, "AT");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    if (!LTE_UART_CheckResponse(response, "OK")) {
        return false;
    }

    /* Disable echo */
    LTE_UART_SendCommand(driver, "ATE0");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    LTE_UART_GetResponse(driver, response, sizeof(response));

    /* Enable full functionality */
    LTE_UART_SendCommand(driver, "AT+CFUN=1");
    if (!LTE_UART_WaitResponse(driver, 2000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    if (!LTE_UART_CheckResponse(response, "OK")) {
        return false;
    }

    /* Check SIM status */
    LTE_UART_SendCommand(driver, "AT+CPIN?");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    LTE_UART_GetResponse(driver, response, sizeof(response));

    if (!LTE_UART_CheckResponse(response, "READY")) {
        return false;
    }

    driver->state = LTE_STATE_READY;
    return true;
}

/**
 * @brief Set phone functionality mode
 */
bool LTE_SetFunctionality(lte_uart_driver_t *driver, lte_cfun_mode_t mode)
{
    char command[32];
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL) {
        return false;
    }

    snprintf(command, sizeof(command), "AT+CFUN=%d", mode);

    LTE_UART_SendCommand(driver, command);
    if (!LTE_UART_WaitResponse(driver, 2000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    return LTE_UART_CheckResponse(response, "OK");
}

/**
 * @brief Disable command echo
 */
bool LTE_DisableEcho(lte_uart_driver_t *driver)
{
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL) {
        return false;
    }

    LTE_UART_SendCommand(driver, "ATE0");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    return LTE_UART_CheckResponse(response, "OK");
}

/**
 * @brief Query device information
 */
bool LTE_QueryDeviceInfo(lte_uart_driver_t *driver)
{
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL) {
        return false;
    }

    /* Get IMEI */
    LTE_UART_SendCommand(driver, "AT+GSN");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (LTE_UART_GetResponse(driver, response, sizeof(response))) {
        LTE_UART_ExtractParameter(response, "", driver->sim_info.imei, sizeof(driver->sim_info.imei));
    }

    return true;
}

/**
 * @brief Get SIM card status
 */
bool LTE_GetSIMStatus(lte_uart_driver_t *driver, lte_sim_status_t *status)
{
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL || status == NULL) {
        return false;
    }

    LTE_UART_SendCommand(driver, "AT+CPIN?");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    if (strstr(response, "READY")) {
        *status = LTE_SIM_READY;
        driver->sim_info.sim_status = LTE_SIM_READY;
        return true;
    } else if (strstr(response, "SIM PIN")) {
        *status = LTE_SIM_PIN_REQUIRED;
        driver->sim_info.sim_status = LTE_SIM_PIN_REQUIRED;
        return true;
    } else if (strstr(response, "SIM PUK")) {
        *status = LTE_SIM_PUK_REQUIRED;
        driver->sim_info.sim_status = LTE_SIM_PUK_REQUIRED;
        return true;
    }

    return false;
}

/**
 * @brief Get IMEI
 */
bool LTE_GetIMEI(lte_uart_driver_t *driver, char *imei, uint8_t max_len)
{
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL || imei == NULL) {
        return false;
    }

    LTE_UART_SendCommand(driver, "AT+GSN");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    /* Extract IMEI (first numeric line) */
    char *start = (char*)response;
    uint8_t len = 0;

    while (len < (max_len - 1) && *start != '\0' && *start != '\r' && *start != '\n') {
        if (*start >= '0' && *start <= '9') {
            imei[len++] = *start;
        }
        start++;
    }

    imei[len] = '\0';

    return (len > 0);
}

/**
 * @brief Get IMSI
 */
bool LTE_GetIMSI(lte_uart_driver_t *driver, char *imsi, uint8_t max_len)
{
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL || imsi == NULL) {
        return false;
    }

    LTE_UART_SendCommand(driver, "AT+CIMI");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    LTE_UART_ExtractParameter(response, "", imsi, max_len);
    strncpy(driver->sim_info.imsi, imsi, sizeof(driver->sim_info.imsi) - 1);

    return true;
}

/**
 * @brief Get phone number
 */
bool LTE_GetPhoneNumber(lte_uart_driver_t *driver, char *phone, uint8_t max_len)
{
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL || phone == NULL) {
        return false;
    }

    LTE_UART_SendCommand(driver, "AT+CNUM");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    LTE_UART_ExtractParameter(response, "+CNUM: ", phone, max_len);
    strncpy(driver->sim_info.phone_number, phone, sizeof(driver->sim_info.phone_number) - 1);

    return true;
}

/**
 * @brief Get network status
 */
bool LTE_GetNetworkStatus(lte_uart_driver_t *driver, lte_network_info_t *info)
{
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL || info == NULL) {
        return false;
    }

    LTE_UART_SendCommand(driver, "AT+CREG?");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    /* Parse +CREG: <n>,<stat>[,<lac>,<ci>[,<AcT>]] */
    int stat = 0;
    sscanf(response, "+CREG: %*d,%d", &stat);
    info->reg_status = (lte_reg_status_t)stat;

    *info = driver->network_info;

    return true;
}

/**
 * @brief Get signal quality
 */
bool LTE_GetSignalQuality(lte_uart_driver_t *driver, lte_signal_quality_t *quality)
{
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL || quality == NULL) {
        return false;
    }

    LTE_UART_SendCommand(driver, "AT+CSQ");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    /* Parse +CSQ: <rssi>,<ber> */
    sscanf(response, "+CSQ: %hhu,%hhu", &quality->rssi, &quality->ber);

    *quality = driver->signal_quality;

    return true;
}

/**
 * @brief Get operator name
 */
bool LTE_GetOperatorName(lte_uart_driver_t *driver, char *operator_name, uint8_t max_len)
{
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL || operator_name == NULL) {
        return false;
    }

    LTE_UART_SendCommand(driver, "AT+COPS?");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    LTE_UART_ExtractParameter(response, "+COPS: ", operator_name, max_len);
    strncpy(driver->network_info.operator_name, operator_name, sizeof(driver->network_info.operator_name) - 1);

    return true;
}

/**
 * @brief Wait for network registration
 */
bool LTE_WaitForRegistration(lte_uart_driver_t *driver, uint32_t timeout_ms)
{
    lte_network_info_t info;
    uint32_t start_time = LTE_GetTicks();

    if (driver == NULL) {
        return false;
    }

    while ((LTE_GetTicks() - start_time) < timeout_ms) {
        if (LTE_GetNetworkStatus(driver, &info)) {
            if (info.reg_status == LTE_REG_REGISTERED_HOME ||
                info.reg_status == LTE_REG_REGISTERED_ROAMING) {
                driver->state = LTE_STATE_REGISTERED;
                return true;
            }
        }
        HAL_Delay(1000);  /* Check every second */
    }

    driver->state = LTE_STATE_SEARCHING;
    return false;
}

/**
 * @brief Send SMS
 */
bool LTE_SendSMS(lte_uart_driver_t *driver, const char *phone_number, const char *message)
{
    char command[64];
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL || phone_number == NULL || message == NULL) {
        return false;
    }

    /* Set SMS text mode */
    LTE_UART_SendCommand(driver, "AT+CMGF=1");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    LTE_UART_GetResponse(driver, response, sizeof(response));

    /* Set recipient */
    snprintf(command, sizeof(command), "AT+CMGS=\"%s\"", phone_number);
    LTE_UART_SendCommand(driver, command);
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    LTE_UART_GetResponse(driver, response, sizeof(response));

    /* Send message text followed by Ctrl-Z */
    HAL_UART_Transmit(driver->huart, (uint8_t*)message, strlen(message), 1000);
    uint8_t ctrl_z = 0x1A;
    HAL_UART_Transmit(driver->huart, &ctrl_z, 1, 100);

    if (!LTE_UART_WaitResponse(driver, 3000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    return LTE_UART_CheckResponse(response, "OK");
}

/**
 * @brief Read SMS
 */
bool LTE_ReadSMS(lte_uart_driver_t *driver, uint8_t index, lte_sms_t *sms)
{
    char command[32];
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL || sms == NULL) {
        return false;
    }

    /* Set SMS text mode */
    LTE_UART_SendCommand(driver, "AT+CMGF=1");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    LTE_UART_GetResponse(driver, response, sizeof(response));

    /* Read SMS */
    snprintf(command, sizeof(command), "AT+CMGR=%u", index);
    LTE_UART_SendCommand(driver, command);
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    sms->message_index = index;

    return true;
}

/**
 * @brief Delete SMS
 */
bool LTE_DeleteSMS(lte_uart_driver_t *driver, uint8_t index)
{
    char command[32];
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL) {
        return false;
    }

    snprintf(command, sizeof(command), "AT+CMGD=%u", index);
    LTE_UART_SendCommand(driver, command);
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    return LTE_UART_CheckResponse(response, "OK");
}

/**
 * @brief Set SMS format (0=PDU, 1=Text)
 */
bool LTE_SetSMSFormat(lte_uart_driver_t *driver, uint8_t format)
{
    char command[32];
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL) {
        return false;
    }

    snprintf(command, sizeof(command), "AT+CMGF=%u", format);
    LTE_UART_SendCommand(driver, command);
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    return LTE_UART_CheckResponse(response, "OK");
}

/**
 * @brief Activate PDP context
 */
bool LTE_ActivatePDP(lte_uart_driver_t *driver, const char *apn)
{
    char command[64];
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL || apn == NULL) {
        return false;
    }

    snprintf(command, sizeof(command), "AT+CGACT=1,1");
    LTE_UART_SendCommand(driver, command);
    if (!LTE_UART_WaitResponse(driver, 2000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    return LTE_UART_CheckResponse(response, "OK");
}

/**
 * @brief Deactivate PDP context
 */
bool LTE_DeactivatePDP(lte_uart_driver_t *driver)
{
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL) {
        return false;
    }

    LTE_UART_SendCommand(driver, "AT+CGACT=0,1");
    if (!LTE_UART_WaitResponse(driver, 2000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    return LTE_UART_CheckResponse(response, "OK");
}

/**
 * @brief Get IP address
 */
bool LTE_GetIPAddress(lte_uart_driver_t *driver, char *ip_addr, uint8_t max_len)
{
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL || ip_addr == NULL) {
        return false;
    }

    LTE_UART_SendCommand(driver, "AT+CGPADDR=1");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    LTE_UART_ExtractParameter(response, "+CGPADDR: ", ip_addr, max_len);

    return true;
}

/**
 * @brief Query PDP status
 */
bool LTE_QueryPDPStatus(lte_uart_driver_t *driver)
{
    char response[LTE_RX_BUFFER_SIZE];

    if (driver == NULL) {
        return false;
    }

    LTE_UART_SendCommand(driver, "AT+CGACT?");
    if (!LTE_UART_WaitResponse(driver, 1000)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, sizeof(response))) {
        return false;
    }

    return true;
}

/**
 * @brief Send generic AT command
 */
bool LTE_SendATCommand(lte_uart_driver_t *driver, const char *command,
                       char *response, uint16_t max_response_len,
                       uint32_t timeout_ms)
{
    if (driver == NULL || command == NULL || response == NULL) {
        return false;
    }

    LTE_UART_SendCommand(driver, command);
    if (!LTE_UART_WaitResponse(driver, timeout_ms)) {
        return false;
    }
    LTE_UART_Process(driver);
    if (!LTE_UART_GetResponse(driver, response, max_response_len)) {
        return false;
    }

    return LTE_UART_CheckResponse(response, "OK");
}

/**
 * @brief Query command (generic)
 */
bool LTE_QueryCommand(lte_uart_driver_t *driver, const char *command_query,
                      char *response, uint16_t max_response_len)
{
    if (driver == NULL || command_query == NULL || response == NULL) {
        return false;
    }

    LTE_UART_SendCommand(driver, command_query);
    if (!LTE_UART_WaitResponse(driver, LTE_CMD_TIMEOUT_MS)) {
        return false;
    }
    LTE_UART_Process(driver);

    return LTE_UART_GetResponse(driver, response, max_response_len);
}

/**
 * @brief Get current driver state
 */
lte_state_t LTE_GetState(lte_uart_driver_t *driver)
{
    if (driver == NULL) {
        return LTE_STATE_ERROR;
    }

    return driver->state;
}

/**
 * @brief Check if module is connected
 */
bool LTE_IsConnected(lte_uart_driver_t *driver)
{
    if (driver == NULL) {
        return false;
    }

    return (driver->state == LTE_STATE_REGISTERED ||
            driver->state == LTE_STATE_READY);
}

/**
 * @brief Check if module is registered to network
 */
bool LTE_IsRegistered(lte_uart_driver_t *driver)
{
    if (driver == NULL) {
        return false;
    }

    return (driver->network_info.reg_status == LTE_REG_REGISTERED_HOME ||
            driver->network_info.reg_status == LTE_REG_REGISTERED_ROAMING);
}

/**
 * @brief Check if module has signal
 */
bool LTE_HasSignal(lte_uart_driver_t *driver)
{
    if (driver == NULL) {
        return false;
    }

    return (driver->signal_quality.rssi > 0 && driver->signal_quality.rssi < 32);
}

/**
 * @brief Convert state to string
 */
const char* LTE_StateToString(lte_state_t state)
{
    switch (state) {
        case LTE_STATE_UNINITIALIZED: return "Uninitialized";
        case LTE_STATE_INITIALIZING: return "Initializing";
        case LTE_STATE_READY: return "Ready";
        case LTE_STATE_NO_SIGNAL: return "No Signal";
        case LTE_STATE_SEARCHING: return "Searching";
        case LTE_STATE_REGISTERED: return "Registered";
        case LTE_STATE_ERROR: return "Error";
        case LTE_STATE_POWER_OFF: return "Power Off";
        default: return "Unknown";
    }
}

/**
 * @brief Convert registration status to string
 */
const char* LTE_RegStatusToString(lte_reg_status_t status)
{
    switch (status) {
        case LTE_REG_NOT_REGISTERED: return "Not Registered";
        case LTE_REG_REGISTERED_HOME: return "Registered (Home)";
        case LTE_REG_SEARCHING: return "Searching";
        case LTE_REG_REGISTRATION_DENIED: return "Registration Denied";
        case LTE_REG_UNKNOWN: return "Unknown";
        case LTE_REG_REGISTERED_ROAMING: return "Registered (Roaming)";
        default: return "Unknown";
    }
}

/**
 * @brief Convert SIM status to string
 */
const char* LTE_SIMStatusToString(lte_sim_status_t status)
{
    switch (status) {
        case LTE_SIM_READY: return "Ready";
        case LTE_SIM_PIN_REQUIRED: return "PIN Required";
        case LTE_SIM_PUK_REQUIRED: return "PUK Required";
        case LTE_SIM_NOT_INSERTED: return "Not Inserted";
        case LTE_SIM_UNKNOWN: return "Unknown";
        default: return "Unknown";
    }
}

/**
 * @brief Get system tick count (platform-dependent)
 * This is a simple wrapper - adapt to your actual timer/tick source
 */
static uint32_t LTE_GetTicks(void)
{
    return HAL_GetTick();
}
