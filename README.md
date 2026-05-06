# LPC1768 CAN Driver

Polling-based CAN communication driver for the NXP LPC1768 (ARM Cortex-M3), with ESP32 companion sketches and a WiFi dashboard for live testing.

## Architecture

```
┌───────────────────────────────────────────────┐
│            Application Layer (demos)          │
│  main_rx_demo / main_tx_demo / sw_filter /    │
│  main_priority_demo / main_test_demo          │
├───────────────────────────────────────────────┤
│            Driver API (can_driver.h)          │
│  can_init · can_transmit · can_receive        │
│  can_set_filter · can_get_status · can_diag   │
├─────────────────────┬─────────────────────────┤
│   HAL (can_hal.c)   │  Ring Buffer            │
│   Clock · Pins ·    │  (can_buffer.h)         │
│   PCLK              │  16-deep FIFO           │
├─────────────────────┴─────────────────────────┤
│      Register Definitions (can_reg.h)         │
│      LPC1768 CAN1/CAN2 hardware registers     │
└───────────────────────────────────────────────┘
```

## Project Structure

```
CAN_Driver/
├── include/                    # Header files
│   ├── can_driver.h            #   Public driver API (init, TX, RX, filter, diag)
│   ├── can_hal.h               #   HAL interface (clock, pins, NVIC)
│   ├── can_reg.h               #   LPC1768 CAN register bit definitions
│   ├── can_buffer.h            #   Circular ring buffer for RX FIFO
│   ├── can_error.h             #   Return/error codes (CAN_OK, CAN_ERR_*)
│   ├── can_gateway.h           #   CAN1↔CAN2 gateway API
│   └── can_test.h              #   Self-test suite API
│
├── src/
│   ├── drivers/
│   │   ├── can_driver.c        #   Core driver (890 lines) — init, TX, RX, filter, ISR, diag
│   │   └── can_gateway.c       #   CAN1↔CAN2 message routing & transform
│   ├── hal/
│   │   └── can_hal.c           #   HAL — PCONP, PCLKSEL, PINSEL, NVIC
│   ├── utils/
│   │   └── can_test.c          #   Test utility functions
│   └── app/                    # Demo applications (copy to main.c to use)
│       ├── main.c              #   Active build target
│       ├── main_rx_demo.c      #   ESP32→LPC receive demo
│       ├── main_tx_demo.c      #   LPC→ESP32 transmit demo
│       ├── sw_filter.c         #   Software acceptance filter demo
│       ├── main_priority_demo.c#   CAN priority/arbitration demo
│       ├── main_test_demo.c    #   Self-test (loopback, no hardware needed)
│       └── main_filter_demo.c  #   Hardware acceptance filter demo
│
├── tests/
│   └── can_test_suite.c        # Full test suite (40+ cases)
│
├── new_code/                   # ESP32 companion sketches (Arduino)
│   ├── CAN_Dashboard/          #   All-in-one WiFi dashboard (RX/TX/Filter/Priority)
│   ├── ESP32_Demo_TX/          #   Multi-ID transmitter for filter & priority tests
│   ├── TX_Test/                #   Simple TX test sketch
│   ├── RX_CAN/                 #   Simple RX test sketch
│   ├── RX_CAN_WiFi/            #   WiFi RX dashboard (standalone)
│   └── main.c                  #   Old standalone LPC main (pre-driver, reference only)
│
├── hex_files/                  # Pre-built hex files for flashing
│   ├── rx_demo.hex
│   ├── tx_demo.hex
│   ├── sw_filter.hex
│   └── Priority_demo.hex
│
├── hardware/
│   └── Hardware Shematic.pdf   # Wiring schematic
│
├── docs/
│   └── driver_audit.md         # Feature coverage audit
│
├── .gitignore
├── README.md
└── driver_audit.md
```

## Build Environment

| Tool | Version |
|------|---------|
| IDE | Keil MDK-ARM µVision 5 |
| Target | NXP LPC1768 (ARM Cortex-M3, 100 MHz) |
| Device Pack | Keil LPC1700_DFP |
| CAN Baud Rate | 500 kbps (PCLK = 25 MHz) |
| ESP32 IDE | Arduino IDE 2.x |

## Driver Features

| Category | Details |
|----------|---------|
| **Init & Config** | `can_init()`, `can_deinit()` — clock, pins, bit timing |
| **Transmission** | `can_transmit()` — Standard (11-bit), Extended (29-bit), RTR |
| **Reception** | `can_receive()` — polling with timeout, ring buffer |
| **Acceptance Filter** | `can_set_filter()`, `can_set_group_filter()`, `can_clear_filters()` |
| **Mode Control** | Normal, Listen-only, Self-test (loopback), Reset |
| **Error Handling** | Stuff, Form, ACK, Bit, CRC, Bus-off detection + auto-recovery |
| **Status** | `can_get_status()` — error counters, bus-off, TX/RX flags |
| **Diagnostics** | `can_get_diag()` — frame counters, arb-lost, overrun, uptime |
| **Callbacks** | `can_register_rx_callback()`, `can_register_error_callback()` |
| **Power Mgmt** | `can_sleep()`, `can_wake()` |
| **Timestamps** | `can_timestamp_tick()` via SysTick (1 ms resolution) |
| **Gateway** | CAN1↔CAN2 routing with ID transform |

## Demo Quick Reference

| Demo | LPC File | ESP32 File | What It Shows |
|------|----------|------------|---------------|
| **RX** | `main_rx_demo.c` | `TX_Test/` or Dashboard TX mode | Receive & display frames |
| **TX** | `main_tx_demo.c` | `RX_CAN/` or Dashboard RX mode | Transmit 8-byte frames |
| **Filter** | `sw_filter.c` | `ESP32_Demo_TX/` or Dashboard Filter mode | Accept/reject by ID |
| **Priority** | `main_priority_demo.c` | `ESP32_Demo_TX/` or Dashboard Priority mode | CAN arbitration |
| **Self-Test** | `main_test_demo.c` | None (loopback) | Loopback TX→RX verify |

### How to run a demo

1. Copy the demo file to `src/app/main.c`
2. Build in Keil (F7) → Flash (F8)
3. Upload the matching ESP32 sketch (or use `CAN_Dashboard`)
4. Open PuTTY on the LPC COM port at **115200 baud**

## WiFi Dashboard (ESP32)

Upload `new_code/CAN_Dashboard/CAN_Dashboard.ino` to the ESP32.

- **WiFi AP:** `CAN_Dashboard` / `canbus123`
- **URL:** `http://192.168.4.1`
- **Modes:** RX Test · TX Test · Filter · Priority (switchable from browser)
- **Library required:** `WebSockets` by Markus Sattler

## Hardware Setup

```
REFER TO DIAGRAM IN HARDWARE FOLDER
```

- **Transceiver:** SN65HVD230 (3V3/5V Tolerant) on both ends
- **Termination:** 120 Ω resistor at each end of the bus
- **Serial debug:** UART0 (P0.2/P0.3) at 115200 baud via USB. You'll need to install CH340 drivers and mbed serial drivers to be installed on windows. While on Linux you can use Screen or Minicom. For MacOS you can use CoolTerm (Not Sure of the whole process)

## API Quick Start

```c
#include "can_driver.h"

/* Initialize */
can_config_t cfg = {
    .channel  = CAN_CHANNEL_1,
    .baudrate = CAN_BAUD_500K,
    .mode     = CAN_MODE_NORMAL,
    .enable_timestamp = true
};
can_init(&cfg);

/* Transmit */
can_message_t tx = { .id = 0x100, .dlc = 4, .data = {0xDE,0xAD,0xBE,0xEF} };
can_transmit(CAN_CHANNEL_1, &tx);

/* Receive (polling) */
can_message_t rx;
if (can_receive(CAN_CHANNEL_1, &rx, 100) == CAN_OK) {
    /* process rx.id, rx.data, rx.dlc */
}

/* Filter */
can_set_filter(CAN_CHANNEL_1, 0x100, 0x7FF, CAN_FRAME_STANDARD);

/* Status */
can_status_t st;
can_get_status(CAN_CHANNEL_1, &st);
```
