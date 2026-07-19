# YOLO OBB Weights

Potato OBB detection weights for the dynamic-cut vision pipeline.

| File | Description |
|------|-------------|
| `best.pt` | Ultralytics YOLO OBB checkpoint |

Load example:

```python
from pathlib import Path
from ultralytics import YOLO

model = YOLO(str(Path(__file__).resolve().parent / "best.pt"))
```
