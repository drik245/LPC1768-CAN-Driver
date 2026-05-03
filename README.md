# LPC1768 CAN Driver

Real-Time CAN Communication Driver for LPC1768-Based Industrial Applications.

## Project Structure
- `src/`        — Driver source files (HAL, driver core, gateway)
- `include/`    — Header files and API
- `tests/`      — On-target test suite (40+ self-test cases)
- `docs/`       — Documentation (API reference, design docs)
- `hardware/`   — Schematics and hardware notes
- `esp_test/`   — ESP32 companion test sketches
- `Reports/`    — Presentation materials
- `Plan/`       — Weekly development plan (PDFs)

## Build Environment
- IDE: Keil MDK-ARM uVision 5
- Target: NXP LPC1768 (ARM Cortex-M3, 100MHz)
- Device Pack: Keil LPC1700_DFP

## Baud Rates Supported
500kbps

## Features
- Dual CAN channel support (CAN1 & CAN2)
- ISR-driven TX/RX with ring buffers
- Hardware acceptance filter (individual + group entries)
- CAN gateway with message routing & transform
- Diagnostics (TX/RX counters, error stats, uptime)
- Bus-off auto-recovery (500ms)
- Sleep/wake power management
- Self-test loopback mode
- Comprehensive test suite (40+ cases)

## Usage
- To be Updated...
