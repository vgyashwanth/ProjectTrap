# Quectel EG800Q LTE Module Driver for STM32C051

## Overview

This is a complete UART-based driver for the Quectel EG800Q-CN LTE module integrated with the STM32C051 microcontroller. The driver provides AT command support for network connectivity, SMS, and data operations.

---

## Hardware Configuration (From Your CubeMX)

### Pin Mapping

| Function | Pin | GPIO Port | Type | Purpose |
|----------|-----|-----------|------|---------|
| **PWRKEY** | PB1 | GPIOB | Digital Out | Power on/off control (active low) |
| **RST** | PA15 | GPIOA | Digital Out | Hardware reset (active low) |
| **PWR_EN** | PB8 | GPIOB | Digital Out | Power supply enable |
| **DCD** | PB3 | GPIOB | Digital In | Data Carrier Detect (from module) |
| **RI** | PB4 | GPIOB | Digital In | Ring Indication (from module) |
| **TX** | PB6 | GPIOB | USART1_TX | UART transmit |
| **RX** | PA10 | GPIOA | USART1_RX | UART receive (default from STM32) |

### UART Configuration

```
- Interface: USART1
- Baud Rate: 115200 bps (default, configurable up to 921600)
- Data Bits: 8
- Stop Bits: 1
- Parity: None
- Flow Control: RTS/CTS (recommended for reliability)
```

---

## Critical Hardware Requirements

### 1. **Level Shifting (MANDATORY)**
```
⚠️  The EG800Q operates at 1.8V logic levels
⚠️  The STM32C051 operates at 3.3V logic levels
⚠️  Without proper level shifting, the module will be damaged or malfunction
```

**Solution 1: Use a Level Shifter IC** (Recommended)
- TXS0101 (Quectel recommended)
- TXS0102
- Other 1.8V ↔ 3.3V bidirectional level shifters

**Solution 2: Voltage Divider** (For UART RX only)
```
STM32 TX → LTE RX (direct 3.3V to 1.8V input, module input tolerates this)

STM32 RX input ← LTE TX (3.3V inputs required):
  LTE_TX → 3k resistor → STM32_RX
  GND   → 2k resistor → STM32_RX
  This creates 1.1V when LTE outputs 1.8V (detected as high at 3.3V interface)
```

### 2. **Power Supply**
- Voltage: 3.3V to 4.3V (typical 3.8V recommended)
- Minimum current: 1.5A average, 2A peak
- **Use a dedicated power supply** for the module, not GPIO power
- Add bulk capacitor: 470µF+ ceramic or 1000µF electrolytic
- Add decoupling capacitors: 100nF on each power rail

### 3. **Antenna**
- Connect a cellular antenna to the antenna pin
- Essential for network registration

---

## File Structure

```
Core/
├── Inc/
│   └── eg800q.h                 # Main driver header
├── Src/
│   ├── eg800q_uart.c            # UART implementation
│   └── eg800q_example.c         # Usage examples
└── ProjectTrapDocuments/
    └── EG800Q_DRIVER_README.md  # This file
```

---

## Driver API Reference

### Initialization

```c
/* Initialize driver with UART handle */
void LTE_UART_Init(lte_uart_driver_t *driver, UART_HandleTypeDef *huart);

/* Run startup sequence */
bool LTE_Startup(lte_uart_driver_t *driver);

/* Deinitialize driver */
void LTE_UART_Deinit(lte_uart_driver_t *driver);
```

### Power Control

```c
void LTE_PowerOn(void);           /* Pull PWRKEY low 500ms to power on */
void LTE_PowerOff(void);          /* Pull PWRKEY low 650ms to power off */
void LTE_HardwareReset(void);     /* Pull RST low 300ms to reset */
```

### Network Operations

```c
bool LTE_WaitForRegistration(lte_uart_driver_t *driver, uint32_t timeout_ms);
bool LTE_GetNetworkStatus(lte_uart_driver_t *driver, lte_network_info_t *info);
bool LTE_GetSignalQuality(lte_uart_driver_t *driver, lte_signal_quality_t *quality);
bool LTE_IsRegistered(lte_uart_driver_t *driver);
bool LTE_HasSignal(lte_uart_driver_t *driver);
```

### SIM/Device Information

```c
bool LTE_GetSIMStatus(lte_uart_driver_t *driver, lte_sim_status_t *status);
bool LTE_GetIMEI(lte_uart_driver_t *driver, char *imei, uint8_t max_len);
bool LTE_GetIMSI(lte_uart_driver_t *driver, char *imsi, uint8_t max_len);
bool LTE_GetPhoneNumber(lte_uart_driver_t *driver, char *phone, uint8_t max_len);
```

### SMS Operations

```c
bool LTE_SendSMS(lte_uart_driver_t *driver, const char *phone_number, const char *message);
bool LTE_ReadSMS(lte_uart_driver_t *driver, uint8_t index, lte_sms_t *sms);
bool LTE_DeleteSMS(lte_uart_driver_t *driver, uint8_t index);
```

### Data Operations (TCP/UDP)

```c
bool LTE_ActivatePDP(lte_uart_driver_t *driver, const char *apn);
bool LTE_DeactivatePDP(lte_uart_driver_t *driver);
bool LTE_GetIPAddress(lte_uart_driver_t *driver, char *ip_addr, uint8_t max_len);
```

### Generic AT Commands

```c
bool LTE_SendATCommand(lte_uart_driver_t *driver, const char *command,
                       char *response, uint16_t max_response_len,
                       uint32_t timeout_ms);

bool LTE_QueryCommand(lte_uart_driver_t *driver, const char *command_query,
                      char *response, uint16_t max_response_len);
```

---

## Integration Steps

### Step 1: Include Header Files

```c
#include "eg800q.h"
```

### Step 2: Create Driver Instance

```c
static lte_uart_driver_t lte_driver;
```

### Step 3: Setup UART Interrupt Handler

In your main.c or USART interrupt handler:

```c
static uint8_t rx_byte;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* Pass byte to LTE driver */
        LTE_UART_RxCallback(&lte_driver, rx_byte);

        /* Process complete messages */
        LTE_UART_Process(&lte_driver);

        /* Re-enable reception */
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

/* In initialization code */
void MX_USART1_UART_Init(void)
{
    /* ... existing HAL initialization ... */

    /* Start receiving data */
    uint8_t rx_byte = 0;
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}
```

### Step 4: Initialize LTE Module

```c
void app_init(void)
{
    /* Initialize driver */
    LTE_UART_Init(&lte_driver, &huart1);

    HAL_Delay(100);

    /* Run startup sequence */
    if (!LTE_Startup(&lte_driver)) {
        /* Handle error */
        return;
    }

    /* Disable echo (optional) */
    LTE_DisableEcho(&lte_driver);
}
```

### Step 5: Use Driver Functions

```c
void app_main_loop(void)
{
    /* Wait for network registration */
    if (LTE_WaitForRegistration(&lte_driver, 120000)) {
        /* Module is registered */

        /* Send SMS */
        LTE_SendSMS(&lte_driver, "+1234567890", "Hello!");

        /* Get signal quality */
        lte_signal_quality_t signal;
        LTE_GetSignalQuality(&lte_driver, &signal);
    }
}
```

---

## Typical Initialization Sequence

```
1. Power supply: Ensure VBAT is stable (3.3-4.3V)
2. LTE_PWR_EN: Set GPIO high to enable voltage regulation
3. LTE_PowerOn(): Calls LTE_PWR_KEY pulse (500ms low)
4. Wait ~2 seconds for VDD_EXT to stabilize
5. Wait ~10 seconds for UART to become active
6. LTE_Startup(): Sends AT commands to verify communication
   - Checks SIM status
   - Enables full functionality mode
7. Module ready for network operations
8. LTE_WaitForRegistration(): Wait for network registration (5-60s)
9. LTE_SendSMS(), LTE_ActivatePDP(), etc.
```

---

## State Diagram

```
UNINITIALIZED
    ↓
INITIALIZING → LTE_Startup()
    ↓
READY → Searching for network
    ↓
NO_SIGNAL / SEARCHING
    ↓
REGISTERED → Ready for data/SMS
    ↓
ERROR → LTE_HardwareReset()
```

---

## Command Response Timeouts

| Operation | Timeout | Notes |
|-----------|---------|-------|
| AT verification | 1s | Quick test command |
| SIM status check | 1s | CPIN status |
| Device info | 1s | IMEI, model |
| Network status | 1s | Registration status |
| Signal quality | 1s | RSSI, BER |
| SMS sending | 3-5s | Depends on network |
| PDP activation | 5-10s | Can be slow |
| Network registration | 5-60s | Depends on signal |

---

## Common Issues and Solutions

### Issue 1: Module Not Responding to AT Commands
**Causes:**
- UART baud rate mismatch
- No level shifter (3.3V signals not being read as 1.8V)
- Module not powered
- PWRKEY pulse too short or wrong polarity

**Solution:**
- Verify USART1 is configured for 115200 baud
- Check level shifter is installed
- Verify VBAT supply voltage (3.3-4.3V)
- Verify PWRKEY pulse timing (500ms minimum)

### Issue 2: "CPIN: NOT INSERTED" or SIM Status Errors
**Causes:**
- SIM card not inserted properly
- SIM card contact dirty
- Wrong SIM card type
- Module not initialized

**Solution:**
- Remove and reinsert SIM card
- Clean contacts with dry cloth
- Wait 10-15 seconds after module boot
- Try alternate SIM card

### Issue 3: Module Powers Off Immediately
**Causes:**
- Insufficient power supply current
- VBAT voltage drops under 3.3V
- Power supply capacitors too small
- PWRKEY timing issue

**Solution:**
- Check VBAT voltage (should be 3.3-4.3V stable)
- Add larger bulk capacitor (470µF+)
- Use separate power supply for module
- Verify PWRKEY pulse (600ms minimum)

### Issue 4: Network Registration Takes Too Long or Fails
**Causes:**
- No signal in area
- SIM card issue
- Antenna not connected
- Wrong APN configuration

**Solution:**
- Check antenna is properly connected
- Move to area with better signal
- Try different location
- Verify SIM card is active with carrier

### Issue 5: Garbled UART Data
**Causes:**
- Baud rate mismatch
- Noise on UART lines
- Improper level shifting
- Loose connections

**Solution:**
- Verify baud rate is 115200 on both sides
- Add 100nF capacitors near signal lines
- Check level shifter connections
- Use shielded cable for UART lines

---

## Performance Considerations

### Memory Usage
```
Driver Structure (lte_uart_driver_t): ~1.2 KB
RX Buffer (512 bytes): 512 bytes
TX Buffer (256 bytes): 256 bytes
Response Buffer (512 bytes): 512 bytes
Total: ~2.5 KB RAM
```

### CPU Usage
- Idle: Minimal (only on UART reception)
- During command: <5% CPU (blocking waits)
- Reception callback: <1ms per byte at 115200 baud

### Power Consumption
- Module idle: ~5-10 mA
- Module active: ~100-200 mA
- Module transmitting: ~500-2000 mA peak
- Sleep mode: <2 mA

---

## Debugging Tips

### Enable UART Echo
```c
LTE_UART_SendCommand(&driver, "ATE1");  /* Enable echo */
LTE_UART_WaitResponse(&driver, 1000);
```

### Monitor Raw UART Data
Use a serial monitor at 115200 baud to see all AT commands and responses.

### Check Module Status
```c
char response[256];
LTE_QueryCommand(&driver, "AT+CFUN?", response, sizeof(response));
/* Should respond: +CFUN: 1 */

LTE_QueryCommand(&driver, "AT+CPIN?", response, sizeof(response));
/* Should respond: +CPIN: READY */
```

### Monitor Ring Indication Pin
Connect LTE_RI (PB4) to LED for visual indication of unsolicited responses.

---

## Files Generated

1. **eg800q.h** - Main header file with type definitions and API
2. **eg800q_uart.c** - UART implementation and command handling
3. **eg800q_example.c** - Example usage patterns
4. **EG800Q_DRIVER_README.md** - This documentation

---

## Version History

- **v1.0** - Initial release with basic AT command support
  - Power management
  - Network registration
  - SMS operations
  - Device information queries
  - Signal quality monitoring
  - PDP context management

---

## License

This driver is provided as-is for use with the ProjectTrap application.

---

## Support and References

**Module Documentation:**
- Quectel EG800Q Series Hardware Design v1.4
- Quectel EG800Q Series AT Command Manual

**Related Files in Project:**
- `Core/Inc/main.h` - Pin definitions
- `ProjectTrap.ioc` - CubeMX configuration
- `ProjectTrapDocuments/quectel_eg800q_series_hardware_design_v1-4.pdf` - Module datasheet

---

## Next Steps

1. Verify all hardware connections and level shifters
2. Test basic AT commands using serial monitor
3. Integrate driver with your application
4. Test each feature (SMS, network registration, data)
5. Configure APN for your carrier
6. Implement error handling and recovery

