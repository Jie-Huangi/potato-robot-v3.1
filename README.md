# Potato Robot — Dynamic Cut v2

[中文版](README.zh.md) | English Version

Documentation and companion assets for the **distance-trigger + encoder-latch + dynamic follow/cut** pipeline implemented by:

```text
/home/jiehuang/potato-robot-v3/code/perception-planning-action/stm32_potato_dynamic_cut_v2.py
```

## 1. Overview

This program synchronizes vision with belt motion using a **hardware encoder latch at camera trigger time**, tracks potatoes into the Delta workspace, and (in cut mode) runs a short-follow cut state machine.

Coordinate chain: `pixel → plane → encoder delta → robot`.

```text
Belt encoder → M850 trigger → Hik camera + M851 latch → YOLO OBB → Target queue → Dynamic pose → M701 follow / cut
```

## 2. Features

- **Distance-triggered capture** — firmware `M850` pulses the camera every N mm; `M852` for manual triggers.
- **Hardware latch alignment** — each frame pops `M851` encoder snapshot `E0` at trigger time.
- **YOLO OBB detection** — background inference so the control loop keeps encoder / M701 timing.
- **Target queue & association** — same-frame merge, cross-frame matching, anchor correction, mileage prune.
- **Encoder Kalman filter** — control velocity from position E (firmware V is log-only).
- **Run modes** — `monitor` / `follow` / `cut`.
- **Cut state machine** — `IDLE → SYNC → CUT_DOWN → HOLD → RECOVER`.
- **Conveyor control** — `M815` / `M802`, confirm-on-start + keyboard toggle.
- **Debug logs** — CSV + annotated frames under the output directory.

## 3. Repository layout

| Path | Description |
|------|-------------|
| [`code/core_dynamic_cut/`](code/core_dynamic_cut/) | Core algorithm excerpts (Chinese comments) |
| [`code/firmware/`](code/firmware/) | STM32F407 flash binaries + English firmware README |
| [`code/yolo_weights/`](code/yolo_weights/) | YOLO OBB weights (`best.pt`) |

## 4. Dependencies

- Python 3, OpenCV, NumPy, Ultralytics YOLO
- Hikvision SDK via `MvCamera.py`
- STM32 USB CDC serial (default `/dev/ttyACM0`, read/write permission required)
- Calibration file `calibration_current.json` (homography, plane→robot, encoder/trigger signs)
- YOLO OBB weights (this repo: [`code/yolo_weights/best.pt`](code/yolo_weights/best.pt))

## 5. How to use

### 5.1 Run the full script

```bash
cd /home/jiehuang/potato-robot-v3/code/perception-planning-action
python3 stm32_potato_dynamic_cut_v2.py
```

There is **no CLI**. Edit parameters in `main()` before launch: `run_mode`, `dry_run_motion`, `model_path`, `conveyor_speed_mm_s`, `work_z_mm` / `cut_z_mm`, etc.

Point `model_path` at this repo’s weights if you prefer:

```python
model_path=Path("/home/jiehuang/potato-robot-v3.1/code/yolo_weights/best.pt")
```

At startup the script loads calibration (if present), applies field direction overrides, and prints a preflight summary.

### 5.2 Interactive safety prompts

| Type | Effect |
|------|--------|
| `HOME` | Confirm then send `G28 S1` homing |
| `WORKZ` | Move to work plane after IK preview |
| `TOOLZERO` / `SKIPTOOL` | Zero end-effector or skip |
| `BELT` | Start conveyor (motion modes when enabled) |

### 5.3 Keyboard while running

| Key | Action |
|-----|--------|
| `q` / Esc | Quit |
| `s` | Save last camera frame |
| `c` | Toggle conveyor |
| `t` | Manual camera trigger (`M852`) |

## 6. Run modes

| Mode | Behavior |
|------|----------|
| `monitor` | Vision + latch + coordinate chain only; no M701 motion; conveyor auto-start disabled |
| `follow` | Send M701 dynamic follow targets in the workspace window |
| `cut` | Full short-follow cut state machine (default in `main()`) |

`dry_run_motion=True` skips arm connection and only validates camera / mapping path.

## 7. Key parameters (in `main()`)

| Parameter | Role |
|-----------|------|
| `trigger_interval_mm` | Belt distance between camera pulses |
| `workspace_limit_x/y_mm` | Safe follow window half-size |
| `cut_station_x_mm` | Nominal cut station X in robot frame |
| `cut_sync_start_distance_mm` / lead | When to lock sync and when to start Z down |
| `follow_feed_mm_s` / `follow_feed_z_mm_s` | XY / Z follow feeds |
| `conveyor_speed_mm_s` | Belt speed when host-controlled |
| `debug_log_enabled` / frames | CSV + image debug output |

## 8. Outputs & troubleshooting

Debug sessions are written under:

```text
output/stm32_potato_dynamic_latch_v2/debug_<timestamp>/
```

Typical CSVs: `events`, `frames`, `detections`, `follow`, `cut`, `timing`.

Common failures:

- Missing calibration → no valid homography / plane→robot
- Serial permission denied on `/dev/ttyACM0`
- Encoder not ready (ID6 stale)
- Latch too old (`max_latch_age_ms`)
- YOLO weights path missing

## 9. Related docs

- Algorithm excerpts: [`code/core_dynamic_cut/README.md`](code/core_dynamic_cut/README.md)
- Firmware flash guide: [`code/firmware/README.md`](code/firmware/README.md)
- Chinese version of this page: [`README.zh.md`](README.zh.md)
