# Potato Robot v3.1

Documentation and code excerpts for the **distance-trigger + encoder-latch + dynamic cut** pipeline.

## Full bilingual docs (default: English)

Open **[README.html](README.html)** in a browser. Use the **EN / 中文** buttons to switch language (English is the default).

## Contents

| Path | Description |
|------|-------------|
| [`code/core_dynamic_cut/`](code/core_dynamic_cut/) | Core algorithm excerpts from `stm32_potato_dynamic_cut_v2.py` (Chinese comments) |
| [`code/固件代码/`](code/固件代码/) | STM32F407 flash binaries + English firmware README |

## Full runnable script (original project)

```text
/home/jiehuang/potato-robot-v3/code/perception-planning-action/stm32_potato_dynamic_cut_v2.py
```

```bash
cd /home/jiehuang/potato-robot-v3/code/perception-planning-action
python3 stm32_potato_dynamic_cut_v2.py
```

Configure `run_mode`, speeds, and cut depths inside `main()` before running. See `README.html` for the full usage guide.
