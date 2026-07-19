# STM32 Delta Robot Firmware (Flash Package)

This folder contains **flash-ready binaries only** (no CubeMX / HAL source tree).

## MCU

| Item | Value |
|------|--------|
| Part number | **STM32F407VET6** (`STM32F407VETx`) |
| Package | LQFP100 |
| Core | ARM Cortex-M4 |
| Flash | 512 KB @ `0x08000000` |
| Host link | USB CDC (typically `/dev/ttyACM0` on Linux) |
| Debug / flash probe | ST-Link (SWD) |

Firmware identity string (query with `M995`): distance-trigger latch family, e.g. `DeltaSTM32 ... distance-trigger-latch-...`.

## What this firmware does

- **USB CDC G/M-code host** for a Delta robot
- **CAN + ZDT closed-loop steppers**: Delta arms (IDs 1–3), end-effector (4), conveyor (5), encoder (6)
- **Delta FK/IK** Cartesian motion (`G0`/`G1`), homing (`G28`), E-stop (`M112`)
- **Follow controller** (`M700`/`M701`/`M702`) for vision-driven XY(/Z) tracking with lookahead / velocity feedforward
- **Conveyor + encoder** (`M815`/`M802`, `M810`–`M812`) for belt speed and distance
- **Camera distance trigger + latch** (`M850`/`M851`/`M852`): GPIO pulse every N mm of belt travel, queue encoder state at trigger time for vision sync

## Files in this folder

| File | Use |
|------|-----|
| `core_stm32f407.hex` | Intel HEX image (common for programmers / OpenOCD) |
| `core_stm32f407.bin` | Raw binary image |
| `core_stm32f407.elf` | ELF with symbols (convenient for OpenOCD `program`) |
| `stlink.cfg` | OpenOCD board/adapter snippet (ST-Link + STM32F4) |

## How to flash (OpenOCD + ST-Link)

From this directory:

```bash
openocd -f stlink.cfg -c "program core_stm32f407.elf verify reset exit"
```

Or with the HEX file:

```bash
openocd -f stlink.cfg -c "program core_stm32f407.hex verify reset exit"
```

Requirements: `openocd`, ST-Link connected via SWD, target powered.

You may also flash `.hex` / `.bin` with STM32CubeProgrammer or similar tools.

## Host connection after flash

1. Connect USB (CDC device appears, e.g. `/dev/ttyACM0`).
2. Ensure your user can read/write the serial device.
3. Send line-based commands ending with newline. Useful checks:
   - `IsDelta`
   - `M995` — firmware version
   - `M913` — homing status after `G28 S1`

## Key G/M-code commands

| Command | Purpose |
|---------|---------|
| `IsDelta` | Identify Delta firmware |
| `M995` | Report firmware version string |
| `G28 S1` | Start limit-switch homing |
| `M913` | Homing state |
| `G90` / `G1` | Absolute mode / Cartesian move |
| `G93` | Report current XYZ |
| `M17` / `M84` | Enable / disable motors |
| `M112` | E-stop |
| `M700` | Configure/enable follow controller (incl. Z feed) |
| `M701` | Set follow target XY(/Z) + optional VX/VY |
| `M702` | Follow status / ErrXY / ErrZ |
| `M812` / `M810` / `M811` | Encoder scale/zero, start sampling, read Enc |
| `M815` / `M802` | Start conveyor (mm/s) / stop conveyor |
| `M820` | Tool (end-effector) zero |
| `M850` | Enable distance camera trigger (interval, pulse, direction) |
| `M851` | Pop camera latch (encoder at trigger) |
| `M852` | Manual trigger + latch |
| `M909` | IK preview (reachability check) |

## Related in this repository

- Core algorithms: [`../core_dynamic_cut/`](../core_dynamic_cut/)
- YOLO weights: [`../yolo_weights/best.pt`](../yolo_weights/best.pt)
