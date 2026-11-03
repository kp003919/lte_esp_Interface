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

--------------------+
|    Application     |   feeds bytes, invokes event/tick
+---------+----------+
│
+---------▼----------+
|      lte_sm        │   states, retries, timeouts, recovery
+----+----------+----+
│          │

+---------▼--+   +---▼-----------+
| lte_tcp    │   │ lte_mqtt      │   command builders
+---------+--+   +---+-----------+
│          │
+----▼----------▼----+
|     lte_driver     │   line assembly, queue, send, power, time
+---------+----------+
│
+---------▼----------+
|      lte_hw        │   UART, GPIO, clock (real or fake)
+--------------------+

```

### Data Flow

1. Byte from modem → `lte_driver_on_rx_byte()`
2. Driver accumulates into CRLF‑terminated line → queued as event
3. Application drains queue → `lte_sm_handle_event(event)`
4. Each loop also calls `lte_sm_handle_event(NULL)` = **tick** → sends next command, checks timeouts, runs recovery
5. State machine transmits via `lte_driver_send_cmd()`

---

## Repository Layout

```

lte-modem-firmware/
├── CMakeLists.txt
├── README.md
├── LICENSE
├── main/
│   ├── lte_types.h           # Shared types: states, events, command IDs
│   ├── lte_hw.h
│   ├── lte_hw.c              # UART/GPIO/clock — real for device, fake for tests
│   ├── lte_driver.h
│   ├── lte_driver.c          # Byte→line assembly, event queue, I/O
│   ├── lte_sm.h
│   ├── lte_sm.c              # State machine, retries, timeouts
│   ├── lte_tcp.h
│   ├── lte_tcp.c             # TCP socket AT command wrappers
│   ├── lte_mqtt.h
│   ├── lte_mqtt.c            # MQTT connect, subscribe helpers
│   └── main.c                # Entry, UART glue, main loop
├── tests/
│   ├── test_lte_hw_fake.h
│   ├── test_lte_hw_fake.c    # Simulated modem + monotonic fake clock
│   ├── test_lte_sm_happy.c   # Full bring‑up path
│   ├── test_lte_sm_errors.c  # Error→recovery transitions
│   ├── test_lte_sm_timeout.c # Timeout detection
│   ├── test_lte_sm_boot.c    # Boot edge cases
│   ├── test_lte_sm_cereg.c   # Registration URC handling
│   └── test_runner.c         # Aggregator + entry point
└── docs/
├── LTE_Test_Suite_Reference.pdf
└── wiring_diagram.png

```

---

## Module Reference

### `lte_sm` — State Machine

Primary public interface:

| Function | Purpose |
|---|---|
| `lte_sm_init()` | Reset to `INIT` state, clear all counters |
| `lte_sm_handle_event(ev)` | Dispatch received line; pass `NULL` for tick |
| `lte_sm_get_state()` | Read current state |
| `lte_sm_set_state(st)` | State transition (internal use) |

*Test‑only:* `lte_sm_force_state()`, `lte_sm_enable_*()`, `lte_sm_disable_*()`

### `lte_driver` — I/O & Queue

| Function | Purpose |
|---|---|
| `lte_driver_on_rx_byte(b)` | Feed one received byte from modem |
| `lte_driver_get_next_event(&ev)` | Dequeue event; returns `false` if empty |
| `lte_driver_send_cmd(str)` | Queue AT command for transmission |
| `lte_driver_power_on()` | Execute power‑on sequence |
| `lte_driver_modem_reset()` | Hard reset the modem |
| `lte_driver_now_ms()` | Get current millisecond timestamp |

> ⚠️ Strings passed to `lte_driver_send_cmd()` must remain valid until transmitted.

### `lte_tcp` / `lte_mqtt` — Protocol Helpers

| Function | Purpose |
|---|---|
| `lte_tcp_open()` | Initiate TCP socket open |
| `lte_mqtt_connect(host, port)` | Start MQTT connection |
| `lte_mqtt_subscribe(topic)` | Subscribe to MQTT topic |

Modem async URC responses (e.g. `+QIOPEN: 0,0`) are normalised to `OK`/`ERROR` by these layers before delivery to the state machine.

### `lte_hw` — Hardware Abstraction

UART, GPIO, and system time. For tests, the clock is simulated and advances only when explicitly moved.

### `lte_types.h` — Shared Definitions

`LteSmState_t`, `LteCommand_t`, `LteEvent_t`  
Event types: `LTE_EVENT_RX_LINE`, `LTE_EVENT_TIMEOUT`, `LTE_EVENT_ERROR`

---

## State Machine

### Happy Path

```

INIT → POWER_ON → WAIT_BOOT → CHECK_SIM → ATTACH → PDP
→ OPEN_SOCKET → MQTT_CONNECT → MQTT_SUBSCRIBE → RUN

```

### Boot Sub‑Sequence

| Step | Command | Transition Trigger | Next State |
|---|---|---|---|
| Handshake | `AT` | `OK` | `WAIT_BOOT` |
| Echo Off | `ATE0` | `OK` | `WAIT_BOOT` |
| SIM Status | `AT+CPIN?` | `+CPIN: READY` + `OK` | `CHECK_SIM` |
| APN Config | `AT+CGDCONT=1,"IP","<APN>"` | `OK` (non‑fatal on error) | `CHECK_SIM` |
| Reg Reporting | `AT+CEREG=2` | `OK` (non‑fatal on error) | `CHECK_SIM` |
| Network Attach | `AT+CGATT=1` | `OK` | `ATTACH` |

No reply within 2 s → re‑send. After 15 attempts → modem reboot.

### Operational States

| State | Action | On `OK` | On Error/Timeout |
|---|---|---|---|
| `ATTACH` | `AT+CGATT=1` | `PDP` | `REATTACH` |
| `PDP` | `AT+CGACT=1,1` | `OPEN_SOCKET` | `REPDP` |
| `OPEN_SOCKET` | `lte_tcp_open()` | `MQTT_CONNECT` | `REOPEN_SOCKET` |
| `MQTT_CONNECT` | `lte_mqtt_connect()` | `MQTT_SUBSCRIBE` | `MQTT_RECONNECT` |
| `MQTT_SUBSCRIBE` | `lte_mqtt_subscribe()` | `RUN` | `MQTT_RESUBSCRIBE` |

### Response Routing

1. `+CEREG:` line → registration handler first
2. Command in flight → command handler
3. State = `ERROR` → select recovery path
4. Recovery state → execute retry
5. Otherwise → forward path (boot sequence)

### Registration (`+CEREG`)

| `stat` | Meaning | Action |
|---|---|---|
| 0 | Not registered | `REATTACH` if ≥ `PDP`; ignored during bring‑up |
| 1, 5 | Registered (home/roaming) | None |
| 2, 4 | Searching/unknown | None |
| 3 | Registration denied | Increment counter → enter `ERROR` |

---

## Failure Handling & Recovery

1. Failure = error reply **or** timeout → increment per‑command counter
2. Success → reset counter to `0`
3. Counter < 3 → dedicated recovery state → resend on next tick
4. Counter = 3 → `ERROR` → `REBOOT_MODEM` → reset modem → clear counters → restart from `AT`
5. Boot states / `RUN` → no dedicated recovery → escalate directly to reboot

```

ATTACH --(1/2 failures)--> REATTACH --(tick: retry)--> ATTACH
ATTACH --(3rd failure)--> ERROR --(tick)--> REBOOT_MODEM --(tick)--> WAIT_BOOT

```

### Timeouts

| Operation | Timeout |
|---|---|
| Network Attach | 30 000 ms |
| PDP Activation | 10 000 ms |
| TCP Open | 8 000 ms |
| MQTT Connect | 8 000 ms |
| MQTT Subscribe | 8 000 ms |
| Boot Retry Interval | 2 000 ms (max 15 attempts) |

Uses unsigned difference for elapsed time — 32‑bit wrap (~49 days) handled safely.

---

## Build, Flash & Run

```bash
# Clone repository
git clone https://github.com/your-username/lte-modem-firmware.git
cd lte-modem-firmware

# Configure — set your APN
export EXTRA_CFLAGS="-DLTE_APN=\"internet\""

# Load ESP‑IDF v5.1+ toolchain
. $HOME/esp/esp-idf/export.sh

# Build
idf.py build

# Flash (update port to match your system)
idf.py -p /dev/ttyUSB0 flash

# Open serial monitor — 115200 baud
idf.py -p /dev/ttyUSB0 monitor
```

**Requirements:**

- Espressif ESP‑IDF v5.1+ with `xtensa-esp32-elf` toolchain
- CMake 3.16 or newer
- AT‑command compatible LTE Cat‑M1/NB‑IoT modem
- Activated Nano‑SIM with data plan; roaming enabled if applicable

---

## Integration Guide

Feed received bytes, drain events, call the tick — that’s the entire interface:

```
#include "lte_sm.h"
#include "lte_driver.h"

void app_init(void)
{
    lte_sm_init();
}

/* Called from UART ISR or byte‑receipt task */
void uart_rx_callback(uint8_t byte)
{
    lte_driver_on_rx_byte(byte);
}

/* Call periodically from main loop */
void app_loop(void)
{
    LteEvent_t ev;

    while (lte_driver_get_next_event(&ev)) {
        lte_sm_handle_event(&ev);
    }
    lte_sm_handle_event(NULL); // Tick: send next, check timeouts

    if (lte_sm_get_state() == LTE_SM_RUN) {
        // ✅ Connection active — safe to publish/send application data
    }
}
```

> 
> ⚠️ **Important:** Do **not** send AT commands directly while the state machine is running — only one outstanding command permitted.

---

## Configuration

Table

| Option | Default | Source | Description |
| --- | --- | --- | --- |
| `LTE_APN` | `"internet"` | Build flag | Carrier APN — most networks reject without correct value |
| `LTE_SM_HONOR_TEST_FLAGS` | `0` | Build flag | `1` = enable test mode overrides |
| `MAX_RETRIES` | `3` | `lte_sm.c` | Consecutive failures before modem reboot |
| `TIMEOUT_*` | See table above | `lte_sm.c` | Per‑operation timeouts in milliseconds |
| `BOOT_STEP_TIMEOUT_MS` | `2000` | `lte_sm.c` | Boot retry interval |
| `BOOT_MAX_SENDS` | `15` | `lte_sm.c` | Max boot attempts before reboot |

> 
> 📌 **MQTT Defaults:** Broker (`broker.hivemq.com:1883`) and topic (`device/commands`) are currently hard‑coded in `lte_sm.c`. Replace with your own broker and topic for production use — see [Roadmap](#roadmap) for moving these to build configuration.

---

## Logging & Example Session

Logging via `SM_LOG()` (defaults to `printf`). State transitions are always logged:

```
STATE -> LTE_SM_POWER_ON (prev=LTE_SM_INIT)
Waiting for modem boot...
Modem boot complete, checking SIM...
SIM is ready
Registration enabled, attaching to network...
STATE -> LTE_SM_ATTACH (prev=LTE_SM_CHECK_SIM)
RX: OK  [state=LTE_SM_ATTACH cmd=1]
STATE -> LTE_SM_PDP (prev=LTE_SM_ATTACH)
Activating PDP context...
Opening TCP socket...
Connecting to MQTT broker...
Subscribing to MQTT topic...
STATE -> LTE_SM_RUN (prev=LTE_SM_MQTT_SUBSCRIBE)
```

**Key failure patterns to watch:**

- `TIMEOUT on command N` — no response within limit
- `Retries exhausted, rebooting modem` — recovery failed; modem reset
- `SIM not ready: +CPIN: SIM PIN` — SIM PIN required
- `URC: registration denied` — network rejected registration

---

## Testing

### Unit Tests (Host)

```
# Build for PC
gcc -std=c11 -Wall -Wextra -I./main tests/*.c main/lte_*.c -o lte_tests

# Run full suite
./lte_tests
```

Output: `TEST SUMMARY: 30/30 passed`

### Test Coverage

Table

| Group | Validates |
| --- | --- |
| Happy Path | Full sequence completes → `RUN` |
| Error Replies | Each error maps to correct recovery state |
| Timeouts | Silence triggers recovery |
| Retry Limits | Exactly 3 failures → reboot path |
| Counter Logic | Success resets count; only consecutive failures count |
| Boot Resilience | 15 retries then reboot; locked SIM blocks; APN/CEREG errors non‑fatal |
| `+CEREG` Handling | Lost/registered/denied transitions |
| `RUN` Behaviour | Failure reboots; stray `OK` messages ignored |

### Hardware‑in‑Loop (HIL)

- **Setup:** BG95‑M3 → USB‑to‑UART → host; power monitoring via shunt
- **Test Cases:** end‑to‑end connect/disconnect; SIM removal/recovery; antenna loss/reattach; delayed replies; invalid APN
- **CI:** Nightly scheduled runs; results attached to workflow

> 
> Simulated tests = **logic correctness**; HIL = **real‑world behaviour**.

---

## Troubleshooting

Table

| Symptom | Likely Cause | What to Check |
| --- | --- | --- |
| Loops in `WAIT_BOOT` → reboot | Modem not answering `AT` | Power capacity, PWRKEY timing, TX/RX swap, baud rate |
| Stuck in `CHECK_SIM` | SIM |

