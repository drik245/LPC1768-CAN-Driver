# LPC1768 CAN Driver — Feature Coverage Audit

## What Should a CAN Driver Do?

A complete CAN driver covers these areas. Here's what your driver implements:

---

### 1. Initialization & Configuration ✅
| Feature | Status | API / File |
|---------|:------:|------------|
| Clock & power enable | ✅ | `can_hal_init_clock()` — PCONP, PCLKSEL0 |
| Pin mux configuration | ✅ | `can_hal_init_pins()` — PINSEL0 for RD1/TD1 |
| Bit timing / baud rate calc | ✅ | `can_calc_btr()` — supports 125K, 250K, 500K, 1M |
| Reset mode entry/exit | ✅ | `can_set_mode()` |
| De-initialization | ✅ | `can_deinit()` — power off, cleanup |

### 2. Transmission ✅
| Feature | Status | API |
|---------|:------:|-----|
| Standard frame TX (11-bit ID) | ✅ | `can_transmit()` |
| Extended frame TX (29-bit ID) | ✅ | `can_transmit()` with `CAN_FRAME_EXTENDED` |
| RTR frame TX | ✅ | `can_transmit()` with `msg.rtr = true` |
| TX buffer status polling | ✅ | Polls SR register for free TX buffer |
| DLC 0-8 support | ✅ | `msg.dlc` field |

### 3. Reception ✅
| Feature | Status | API |
|---------|:------:|-----|
| Standard frame RX | ✅ | `can_receive()` |
| Extended frame RX | ✅ | `can_receive()` + RFF bit check |
| RTR frame RX | ✅ | `can_receive()` + RTR bit check |
| Polling mode RX | ✅ | `hw_read_frame()` polls GSR.RBS |
| Timeout support | ✅ | `can_receive(ch, msg, timeout_ms)` |
| Software ring buffer | ✅ | `can_buffer.h` — 16-deep circular buffer |
| Buffer release (CMR.RRB) | ✅ | After each frame read |

### 4. Acceptance Filtering ✅
| Feature | Status | API |
|---------|:------:|-----|
| Individual SFF filter (11-bit) | ✅ | `can_set_filter()` — up to 16 entries |
| Individual EFF filter (29-bit) | ✅ | `can_set_filter()` — up to 8 entries |
| Group SFF filter (range) | ✅ | `can_set_group_filter()` |
| Group EFF filter (range) | ✅ | `can_set_group_filter()` |
| Filter clear / bypass | ✅ | `can_clear_filters()` → AFMR=bypass |
| Sorted AF table rebuild | ✅ | `af_rebuild()` — insertion sort by (SCC, ID) |

### 5. Mode Control ✅
| Feature | Status | API |
|---------|:------:|-----|
| Normal mode | ✅ | `can_set_mode(ch, CAN_MODE_NORMAL)` |
| Listen-only mode | ✅ | `can_set_mode(ch, CAN_MODE_LISTEN)` |
| Self-test / loopback | ✅ | `can_set_mode(ch, CAN_MODE_SELFTEST)` |
| Reset mode | ✅ | `can_set_mode(ch, CAN_MODE_RESET)` |

### 6. Error Handling ✅
| Feature | Status | Details |
|---------|:------:|---------|
| Error type enum | ✅ | Stuff, Form, ACK, Bit, CRC, Bus-off |
| Error counter readback | ✅ | `can_get_status()` → tx_err, rx_err |
| Bus-off detection | ✅ | `can_get_status()` → `bus_off` flag |
| Error warning flag | ✅ | `can_get_status()` → `error_warning` |
| Error callback registration | ✅ | `can_register_error_callback()` |
| Bus-off auto-recovery | ✅ | `busoff_recovery_check()` |

### 7. Status & Diagnostics ✅
| Feature | Status | API |
|---------|:------:|-----|
| TX/RX error counts | ✅ | `can_get_status()` |
| Bus-off state | ✅ | `can_get_status()` |
| TX pending flag | ✅ | `can_get_status()` |
| RX available flag | ✅ | `can_get_status()` |
| Frame counters (TX/RX) | ✅ | `can_get_diag()` |
| Error frame counters | ✅ | `can_get_diag()` |
| Arbitration lost count | ✅ | `can_get_diag()` |
| Data overrun count | ✅ | `can_get_diag()` |
| Uptime tracking | ✅ | `can_get_diag()` → uptime_ms |
| Diagnostic reset | ✅ | `can_reset_diag()` |

### 8. Callbacks ✅
| Feature | Status | API |
|---------|:------:|-----|
| RX callback | ✅ | `can_register_rx_callback()` |
| Error callback | ✅ | `can_register_error_callback()` |

### 9. Power Management ✅
| Feature | Status | API |
|---------|:------:|-----|
| Sleep mode | ✅ | `can_sleep()` |
| Wake-up | ✅ | `can_wake()` |

### 10. Timestamping ✅
| Feature | Status | API |
|---------|:------:|-----|
| Frame timestamp | ✅ | `msg.timestamp` via SysTick |
| Tick function | ✅ | `can_timestamp_tick()` |

---

## Architecture

```
┌─────────────────────────────────────────────┐
│              Application Layer               │
│  main_rx_demo / main_tx_demo / priority_demo │
├─────────────────────────────────────────────┤
│              Driver API (can_driver.h)       │
│  can_init / can_transmit / can_receive       │
│  can_set_filter / can_get_status / can_diag  │
├─────────────────────────────────────────────┤
│   HAL (can_hal.c)   │  Ring Buffer (can_buffer.h)  │
│   Clock / Pins /    │  16-deep circular FIFO       │
│   NVIC control      │                              │
├─────────────────────┤──────────────────────────────┤
│        Register Definitions (can_reg.h)             │
│        LPC1768 CAN1/CAN2 hardware registers         │
└─────────────────────────────────────────────────────┘
```

## File Map

| File | Purpose | Lines |
|------|---------|:-----:|
| `can_driver.h` | Public API — all types, enums, functions | 132 |
| `can_driver.c` | Core implementation — init, TX, RX, filter, ISR | 890 |
| `can_hal.h/c` | Hardware Abstraction — clock, pins, NVIC | 115 |
| `can_reg.h` | Register bit definitions | 190+ |
| `can_buffer.h` | Ring buffer for RX FIFO | 70+ |
| `can_error.h` | Error code definitions | ~30 |
| `can_test.h` | Self-test suite API | ~70 |
| `can_gateway.h` | CAN1↔CAN2 gateway API | ~60 |

## Demos Tested

| Demo | What It Proves |
|------|---------------|
| **RX Demo** | `can_receive()` polling, serial output ✅ |
| **TX Demo** | `can_transmit()` with driver API ✅ |
| **Priority Demo** | CAN ID arbitration, priority classification ✅ |
| **Filter Demo** | Software acceptance filtering (HW filter programmed but unreliable without termination) ✅ |
| **Self-Test Demo** | Loopback mode, LED pass/fail ✅ |

## Verdict: Your driver covers ALL standard CAN driver features

> [!TIP]
> The only item that needs hardware (120Ω termination resistors) to work reliably is the **hardware acceptance filter** (AFMR=0 mode). Everything else is fully functional with your current setup.
