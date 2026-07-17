# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Arduino (Mega 2560) firmware for controlling an insulating-oil immersion heater. It reads inlet/outlet oil temperatures via two MAX31856 thermocouple amplifiers, drives an external SCR power controller (Pion PION-L1W-025-01) over Modbus RTU as a Modbus master, exposes its own process values as a Modbus RTU slave for a PC/LabVIEW client, and drives a Nextion HMI touchscreen over a plain serial text protocol. Control law is feedforward + PID: PID drives outlet temperature to setpoint, and a feedforward term compensates for the inlet temperature.

The entire application is one file: [src/main.cpp](src/main.cpp).

## Build / upload / test commands

This is a PlatformIO project (see [platformio.ini](platformio.ini)), env `megaatmega2560` (`atmelavr` platform, `arduino` framework). Use the PlatformIO CLI (`pio`):

```
pio run                       # build
pio run --target upload       # build and flash to the connected Mega 2560
pio device monitor -b 19200   # open serial monitor (DEBUG_SERIAL runs at 19200 baud)
pio test                      # run tests in test/ (currently empty — no tests exist yet)
```

There is no separate lint step configured; compiler warnings from `pio run` are the only static check.

## Hardware / serial architecture

Four hardware UARTs are each dedicated to one role, defined at the top of `main.cpp`:

- `DEBUG_SERIAL` (`Serial`) — USB serial monitor for debugging.
- `HMI_SERIAL` (`Serial1`) — Nextion HMI display.
- `MASTER_SERIAL` (`Serial2`) — RS485 bus where this board is Modbus **master**, polling the SCR power controller.
- `SLAVE_SERIAL` (`Serial3`) — RS485 bus where this board is Modbus **slave**, polled by an external PC/LabVIEW client.

All four are initialized to the same baud rate (`BAUD_RATE = 19200`, 8N1). Changing the baud rate requires updating all four peers (HMI, SCR controller, and the PC/LabVIEW client), not just the firmware.

### Modbus slave register map (`holdingRegs[10]`)

Exposed to the external PC client via `ModbusRTUSlave`, unit ID `SLAVE_UNIT_ID = 1`:

| Index | Meaning |
|---|---|
| 0 | In_T — inlet oil temperature (°C) |
| 1 | Out_T — outlet oil temperature (°C) |
| 2 | Power — heater power (kW) |
| 3 | SCR RUN/STOP (write) |
| 4 | Set_T — target temperature (°C) (write) |

Only 5 of the 10 allocated registers are currently wired up; the array is oversized deliberately to leave headroom.

### Modbus master target (SCR power controller)

Addressed as slave `ID_SCR = 1` over `MASTER_SERIAL`. Relevant registers on the Pion controller (see constants near the top of `main.cpp`): `ADDR_RUN_CTRL` (coil, unused — controller is left in default RUN), `ADDR_S_PHASE_CTRL` (holding register, write-only control value 0–10000 representing 0–100% output), `ADDR_S_CURRENT_SA` (input register, read-only current in 0.1 A steps, used to derive `heaterPower`).

### Nextion HMI protocol

Outbound (`sendToNextion`): text commands of the form `<componentId>.val=<int>` terminated by three `0xFF` bytes — the Nextion wire protocol. Component IDs currently in use: `x0` (In_T), `x1` (Out_T), `x2` (heaterPower), `x3` (Set_T, sent once at startup). Temperatures are sent as `value * 10` because the Nextion side displays one decimal place.

Inbound (`checkHMI` / `parseNextionPacket`): a small hand-rolled framer that buffers bytes into `hmi_buffer` until three consecutive `0xFF` terminator bytes are seen, then parses two fixed formats: a `"START"` packet (name + 4-byte little-endian int for the new setpoint, sets `StartStop = 1`) and a `"STOP"` packet (sets `StartStop = 0`). If you add a new HMI button/control, follow this same length-prefixed-by-terminator pattern and extend `parseNextionPacket`.

### Control loop

`loop()` order matters: `slave.poll()` (service PC requests) → `checkHMI()` (drain HMI input) → every `sensorReadInterval` (1000 ms) read sensors and, if `StartStop == 1`, run PID (`Input = Out_T`, `Setpoint = Set_T`) and add a feedforward term `(Set_T - In_T) * alpha` before writing the combined value to the SCR controller, clamped to `[0, 10000]`. When stopped, it explicitly writes 0 to the SCR controller rather than leaving the last value in place.

`Kp`, `Ki`, `Kd`, and `alpha` (feedforward gain) are marked in comments as needing tuning — treat existing values as placeholders, not calibrated constants.

## Conventions in this codebase

- Comments and log/debug strings are written in Korean; keep new comments consistent with that.
- Debug/serial-monitor prints throughout `readSensors()` and the Modbus write path are commented out rather than deleted (kept for re-enabling during hardware bring-up) — follow this pattern instead of removing them outright when adding new debug output.
