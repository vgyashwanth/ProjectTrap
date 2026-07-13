/**
 ******************************************************************************
 * @file           : eg800q.h
 * @brief          : Quectel EG800Q Series LTE Module Driver Header
 * @attention
 * Unified driver for Quectel EG800Q-CN LTE module
 * Supports UART interface (115200 baud default) with AT commands
 * Based on Quectel EG800Q-Series Hardware Design v1.4
 * IMPORTANT: Module operates at 1.8V logic levels - level shifter required!
 ******************************************************************************
 */

#ifndef EG800Q_H
#define EG800Q_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* ============================================================================
 * AT COMMAND PROTOCOL DEFINITIONS
 * ============================================================================ */

/* AT Command Terminator */
#define AT_CMD_TERMINATOR       "\r\n"
#define AT_CTRL_Z               0x1A

/* Buffer Sizes */
#define LTE_RX_BUFFER_SIZE      512
#define LTE_TX_BUFFER_SIZE      256
#define LTE_CMD_TIMEOUT_MS      5000

/* ============================================================================
 * UART INTERFACE SPECIFICATIONS
 * ============================================================================ */

/* UART Configuration */
#define EG800Q_BAUD_RATE        115200      /* Default baud rate */
#define EG800Q_DATA_BITS        8
#define EG800Q_STOP_BITS        1
#define EG800Q_PARITY           0           /* None */
#define EG800Q_FLOW_CONTROL     1           /* RTS/CTS enabled */

/* UART Voltage Levels */
#define EG800Q_LOGIC_VOLTAGE    1800        /* 1.8V (mV) */

/* ============================================================================
 * COMMON DATA STRUCTURES
 * ============================================================================ */

/* LTE Module States */
typedef enum {
    LTE_STATE_UNINITIALIZED = 0,
    LTE_STATE_INITIALIZING,
    LTE_STATE_READY,
    LTE_STATE_NO_SIGNAL,
    LTE_STATE_SEARCHING,
    LTE_STATE_REGISTERED,
    LTE_STATE_ERROR,
    LTE_STATE_POWER_OFF
} lte_state_t;

/* Network Registration States */
typedef enum {
    LTE_REG_NOT_REGISTERED = 0,
    LTE_REG_REGISTERED_HOME = 1,
    LTE_REG_SEARCHING = 2,
    LTE_REG_REGISTRATION_DENIED = 3,
    LTE_REG_UNKNOWN = 4,
    LTE_REG_REGISTERED_ROAMING = 5
} lte_reg_status_t;

/* SIM Card Status */
typedef enum {
    LTE_SIM_READY = 0,
    LTE_SIM_PIN_REQUIRED = 1,
    LTE_SIM_PUK_REQUIRED = 2,
    LTE_SIM_NOT_INSERTED = 3,
    LTE_SIM_UNKNOWN = 4
} lte_sim_status_t;

/* CFUN (Phone Functionality) Modes */
typedef enum {
    LTE_CFUN_DISABLED = 0,           /* Minimum functionality */
    LTE_CFUN_ENABLED = 1,            /* Full functionality */
    LTE_CFUN_RFONLY = 2,             /* RF disabled, but USB enabled */
    LTE_CFUN_AIRPLANE_MODE = 4       /* Airplane mode */
} lte_cfun_mode_t;

/* Signal Quality Information */
typedef struct {
    uint8_t rssi;                   /* Received Signal Strength Indicator (0-31, 99=unknown) */
    uint8_t ber;                    /* Bit Error Rate (0-7, 99=unknown) */
    int8_t rsrp;                    /* Reference Signal Received Power (dBm) */
    int8_t rsrq;                    /* Reference Signal Received Quality (dB) */
    int16_t rssnr;                  /* Reference Signal Signal-to-Noise Ratio (dB) */
} lte_signal_quality_t;

/* Network Information */
typedef struct {
    lte_reg_status_t reg_status;    /* Registration status */
    uint16_t lac;                   /* Location Area Code */
    uint32_t ci;                    /* Cell ID */
    uint8_t act;                    /* Access Technology (0=GSM, 2=UTRAN, 7=LTE) */
    char operator_name[32];         /* Operator name */
    char mcc_mnc[6];                /* Mobile Country Code + Mobile Network Code */
} lte_network_info_t;

/* SIM Information */
typedef struct {
    lte_sim_status_t sim_status;    /* SIM status */
    char imei[16];                  /* International Mobile Equipment Identity */
    char imsi[16];                  /* International Mobile Subscriber Identity */
    char phone_number[20];          /* Phone number */
} lte_sim_info_t;

/* SMS Message Structure */
typedef struct {
    uint8_t message_index;          /* SMS index in storage */
    char sender[20];                /* Sender phone number */
    char timestamp[20];             /* Timestamp: "YY/MM/DD,HH:MM:SS±ZZ" */
    char message[160];              /* Message content */
    uint16_t message_length;        /* Message length in bytes */
} lte_sms_t;

/* ============================================================================
 * UART INTERFACE DEFINITIONS
 * ============================================================================ */

/* LTE UART Driver Context Structure */
typedef struct {
    UART_HandleTypeDef *huart;      /* UART handle */
    lte_state_t state;              /* Current state */
    uint8_t rx_buffer[LTE_RX_BUFFER_SIZE];    /* Receive buffer */
    uint8_t tx_buffer[LTE_TX_BUFFER_SIZE];    /* Transmit buffer */
    uint16_t rx_index;              /* Current RX buffer index */
    uint16_t rx_length;             /* Received data length */
    bool new_response_available;    /* Flag: new response ready */
    bool command_in_progress;       /* Flag: waiting for response */
    uint32_t last_command_time;     /* Timestamp of last command */

    /* Cached data */
    lte_sim_info_t sim_info;        /* SIM information */
    lte_network_info_t network_info; /* Network information */
    lte_signal_quality_t signal_quality; /* Signal quality */

    /* Response buffer */
    uint8_t response_buffer[LTE_RX_BUFFER_SIZE];
    uint16_t response_length;
} lte_uart_driver_t;

/* ============================================================================
 * DRIVER INITIALIZATION AND DEINITIALIZATION
 * ============================================================================ */

void LTE_UART_Init(lte_uart_driver_t *driver, UART_HandleTypeDef *huart);
void LTE_UART_Deinit(lte_uart_driver_t *driver);

/* ============================================================================
 * HARDWARE CONTROL FUNCTIONS
 * ============================================================================ */

/* Power and Reset Control */
void LTE_PowerOn(void);            /* Pull PWRKEY low for 500ms to power on */
void LTE_PowerOff(void);           /* Pull PWRKEY low for 650ms to power off */
void LTE_HardwareReset(void);      /* Pull RESET_N low for 300ms to reset */
void LTE_SetFlowControl(uint8_t enable);  /* Enable/disable RTS/CTS flow control */

/* ============================================================================
 * UART COMMUNICATION FUNCTIONS
 * ============================================================================ */

/* Low-level communication */
void LTE_UART_SendCommand(lte_uart_driver_t *driver, const char *command);
void LTE_UART_RxCallback(lte_uart_driver_t *driver, uint8_t data);
void LTE_UART_Process(lte_uart_driver_t *driver);
bool LTE_UART_WaitResponse(lte_uart_driver_t *driver, uint32_t timeout_ms);
bool LTE_UART_GetResponse(lte_uart_driver_t *driver, char *response, uint16_t max_len);

/* ============================================================================
 * INITIALIZATION AND CONFIGURATION
 * ============================================================================ */

bool LTE_Startup(lte_uart_driver_t *driver);
bool LTE_SetFunctionality(lte_uart_driver_t *driver, lte_cfun_mode_t mode);
bool LTE_DisableEcho(lte_uart_driver_t *driver);
bool LTE_QueryDeviceInfo(lte_uart_driver_t *driver);

/* ============================================================================
 * SIM CARD OPERATIONS
 * ============================================================================ */

bool LTE_GetSIMStatus(lte_uart_driver_t *driver, lte_sim_status_t *status);
bool LTE_GetIMEI(lte_uart_driver_t *driver, char *imei, uint8_t max_len);
bool LTE_GetIMSI(lte_uart_driver_t *driver, char *imsi, uint8_t max_len);
bool LTE_GetPhoneNumber(lte_uart_driver_t *driver, char *phone, uint8_t max_len);

/* ============================================================================
 * NETWORK OPERATIONS
 * ============================================================================ */

bool LTE_GetNetworkStatus(lte_uart_driver_t *driver, lte_network_info_t *info);
bool LTE_GetSignalQuality(lte_uart_driver_t *driver, lte_signal_quality_t *quality);
bool LTE_GetOperatorName(lte_uart_driver_t *driver, char *operator_name, uint8_t max_len);
bool LTE_WaitForRegistration(lte_uart_driver_t *driver, uint32_t timeout_ms);

/* ============================================================================
 * SMS OPERATIONS
 * ============================================================================ */

bool LTE_SendSMS(lte_uart_driver_t *driver, const char *phone_number, const char *message);
bool LTE_ReadSMS(lte_uart_driver_t *driver, uint8_t index, lte_sms_t *sms);
bool LTE_DeleteSMS(lte_uart_driver_t *driver, uint8_t index);
bool LTE_SetSMSFormat(lte_uart_driver_t *driver, uint8_t format);  /* 0=PDU, 1=Text */

/* ============================================================================
 * DATA OPERATIONS (TCP/UDP)
 * ============================================================================ */

bool LTE_ActivatePDP(lte_uart_driver_t *driver, const char *apn);
bool LTE_DeactivatePDP(lte_uart_driver_t *driver);
bool LTE_GetIPAddress(lte_uart_driver_t *driver, char *ip_addr, uint8_t max_len);
bool LTE_QueryPDPStatus(lte_uart_driver_t *driver);

/* ============================================================================
 * AT COMMAND WRAPPERS (Generic)
 * ============================================================================ */

bool LTE_SendATCommand(lte_uart_driver_t *driver, const char *command,
                       char *response, uint16_t max_response_len,
                       uint32_t timeout_ms);

bool LTE_QueryCommand(lte_uart_driver_t *driver, const char *command_query,
                      char *response, uint16_t max_response_len);

/* ============================================================================
 * STATE AND STATUS QUERIES
 * ============================================================================ */

lte_state_t LTE_GetState(lte_uart_driver_t *driver);
bool LTE_IsConnected(lte_uart_driver_t *driver);
bool LTE_IsRegistered(lte_uart_driver_t *driver);
bool LTE_HasSignal(lte_uart_driver_t *driver);

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

const char* LTE_StateToString(lte_state_t state);
const char* LTE_RegStatusToString(lte_reg_status_t status);
const char* LTE_SIMStatusToString(lte_sim_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* EG800Q_H */
