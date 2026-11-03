# LTE Modem Firmware

Firmware for connecting an embedded device to the internet over LTE. It powers up an LTE modem, brings it through SIM check, network attach and data session, opens a TCP socket, connects to an MQTT broker, and keeps the connection running — recovering automatically when something fails.

The project has two integrated components:

- **Firmware:** layered C implementation (hardware → driver → protocol helpers → state machine) controlling the modem via AT commands.
- **Test harness:** simulated modem allowing every success, error, and timeout path to be validated without physical hardware.

---

## Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Hardware Configuration](#hardware-configuration)
4. [Architecture](#architecture)
5. [Repository Layout](#repository-layout)
6. [Module Reference](#module-reference)
7. [State Machine](#state-machine)
8. [Failure Handling & Recovery](#failure-handling--recovery)
9. [Build, Flash & Run](#build-flash--run)
10. [Integration Guide](#integration-guide)
11. [Configuration](#configuration)
12. [Logging & Example Session](#logging--example-session)
13. [Testing](#testing)
14. [Troubleshooting](#troubleshooting)
15. [Known Limitations](#known-limitations)
16. [Roadmap](#roadmap)
17. [Project Status & License](#project-status--license)

---

## Overview

An LTE modem is controlled via AT commands. Each command returns a result (`OK` or error), and the modem may send unsolicited reports such as network status changes. Bringing up a connection is a multi‑step sequence — modem ready → SIM valid → network registered → data session active → application connected — and every step can fail independently.

This project manages the sequence with a deterministic state machine following one core rule: **only one command in flight at a time**. Retry limits, per‑command timeouts, and escalation to modem reset prevent deadlock.

---

## Features

- Full bring‑up sequence: power‑on → boot handshake → SIM validation → APN setup → network attach → PDP context → TCP socket → MQTT connect/subscribe → connected run state
- Strict command sequencing: one outstanding command prevents misattributed responses
- Per‑command retry counters (max 3 consecutive failures; reset on success)
- Explicit timeouts for every operation; periodic tick detects unresponsive modem
- Automatic recovery: retry → dedicated recovery state → reboot after exhaustion
- Boot sequence with independent timer: retry every 2 s, up to 15 attempts
- Registration handling: `+CEREG` URC and query forms
- Unified error recognition: `ERROR`, `+CME ERROR`, `+CMS ERROR`
- Layered design: state machine is hardware‑agnostic
- 30 host‑side unit tests with deterministic fake clock and scripted responses

---

## Hardware Configuration

| Item | Value |
|---|---|
| Microcontroller / Board | Espressif ESP32‑WROOM‑32 DevKitC‑V4 |
| LTE Modem Module | Quectel BG95‑M3; Firmware: BG95M3MAR02A04 |
| SIM & Operator | Vodafone UK; APN: `internet`; SIM PIN: none |
| Modem UART | UART2; TX=GPIO17, RX=GPIO16; 115200 baud, 8N1 |
| Control Pins | PWRKEY=GPIO4, RESET_N=GPIO5, STATUS=GPIO2 |
| Power Supply | 5 V / 2 A regulated; peak transmit ~280 mA @ 23 dBm |
| Antenna | Quectel 3G/LTE monopole — mounted clear of metal and PCB |

### Wiring Connections

| ESP32 Pin | Modem Pin | Function |
|---|---|---|
| GPIO16 | TXD | Modem → MCU UART Receive |
| GPIO17 | RXD | MCU → Modem UART Transmit |
| GPIO4 | PWRKEY | Power‑on Control |
| GPIO5 | RESET_N | Active‑low Reset |
| GPIO2 | STATUS | Modem Status Output |
| 3.3 V | VDDIO | Modem I/O Power |
| GND | GND | Common Ground — single‑point connection |
| 5 V / 2 A | VCC | Modem Main Supply — **⚠️ Do NOT power from ESP32 3.3 V rail** |

---

## Architecture

The state machine never touches hardware directly. Power, reset, and timing are abstracted through the driver and hardware layers.

