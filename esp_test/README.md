# ESP32 CAN Test Suite — LPC1768 Driver Companion

Test sketches for verifying the LPC1768 CAN driver using an ESP32 board.

## Hardware Required

| Component | Purpose |
|---|---|
| ESP32 Dev Board | Test controller |
| SN65HVD230 or MCP2551 | CAN Transceiver (3.3 V compatible) |
| 120 Ω Resistor (×2) | Bus termination at each end |
| Jumper wires | Connections |

## Wiring

```
ESP32              SN65HVD230         LPC1768 Board
─────              ──────────         ─────────────
GPIO 5  ────────►  TX (D)
GPIO 4  ◄────────  RX (R)
3.3V    ────────►  VCC                    ┌──── CANH
GND     ────────►  GND                    │
                   CANH ─────── Bus ──────┤
                   CANL ─────── Bus ──────┘──── CANL
                                │
                             120 Ω (each end)
```

## Sketches

### 1. `can_loopback_test/` — Run This First!
**No external hardware needed.** Tests the ESP32's TWAI controller in internal
loopback mode. Verifies:
- Standard & extended frame TX/RX
- Data integrity (CAFEBABE, DEADBEEF patterns)
- Zero-length and max-ID edge cases
- 8-message burst

Upload → open Serial Monitor at 115200 → all tests should PASS.

### 2. `can_transmitter/` — Send to LPC1768
Sends test CAN messages that exercise the LPC1768 gateway routes:

| Test | ID | Expect on LPC1768 |
|---|---|---|
| Route 1 msg | `0x150` | Gateway forwards CAN1→CAN2 |
| Route 2 msg | `0x250` | Gateway forwards CAN2→CAN1 |
| Out-of-range | `0x400` | Gateway drops (no matching route) |
| Heartbeat | `0x700` | Status frame |
| Burst ×8 | `0x100–0x107` | Stresses RX buffer / ISR |

### 3. `can_receiver/` — Listen to LPC1768
Runs in **listen-only mode** (no ACK interference). Classifies every received
frame by gateway route and prints a formatted table with periodic diagnostics.

## Quick Start

```bash
# 1. Install ESP32 board support in Arduino IDE
#    Board Manager → search "esp32" → install by Espressif

# 2. Select board: "ESP32 Dev Module"

# 3. Upload can_loopback_test first (verify ESP32 works)

# 4. Wire the transceiver, upload can_transmitter or can_receiver
```

## Baud Rate

All sketches default to **500 kbps** to match the LPC1768 `main.c` demo:
```c
can_config_t cfg1 = { CAN_CHANNEL_1, CAN_BAUD_500K, ... };
```

Change `TWAI_TIMING_CONFIG_500KBITS()` to `_250KBITS()` / `_125KBITS()` / `_1MBITS()` if needed.

## GPIO Pins

Default: `TX = GPIO 5`, `RX = GPIO 4`. Change in each sketch:
```cpp
#define CAN_TX_PIN   GPIO_NUM_5
#define CAN_RX_PIN   GPIO_NUM_4
```
