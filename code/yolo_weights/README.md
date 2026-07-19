# YOLO OBB Weights

Potato OBB detection weights used by `stm32_potato_dynamic_cut_v2.py`.

| File | Description |
|------|-------------|
| `best.pt` | Ultralytics YOLO OBB checkpoint from `exp-final` |

Source (original project):

```text
/home/jiehuang/potato-robot-v3/code/perception-planning-action/ultralytics/runs/obb/runs/obb/exp-final/weights/best.pt
```

In `main()`, set:

```python
model_path=Path("/home/jiehuang/potato-robot-v3.1/code/yolo_weights/best.pt")
```
