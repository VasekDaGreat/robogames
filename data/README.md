# Measurements: Hokuyo URG-04LX Lidar

Lidar scans recorded for LEGO brick detection in the "Robot uklízeč" arena (see [competition rules](../competition%20docs/comp%20rules.md)).

* [Lidar source code](../code/lidar) (includes a detailed description of the output format)

## Measurements

Both measurements were taken on 2026-11-09 with `urg04lx.py` in continuous mode
(5 s, ~10 scans/s, 51 scans each, full range of steps 44–725 ≈ −120° to +120°).

| Folder | Scene | Valid points | Distance range |
|---|---|---|---|
| [measurement_6_bricks](measurement_6_bricks) | 6 LEGO Duplo bricks, as at the start of a round | 12 317 / 34 782 (35 %) | 206–5393 mm |
| [measurement_with_box](measurement_with_box) | bricks with the target box (21 × 21 × 10 cm, backboard ≥ 30 cm) | 14 888 / 34 782 (43 %) | 203–5415 mm |

Recorded with:

    python urg04lx.py --port COM6 --mode time --seconds 5 --out measurement_6_bricks

## Files in each folder

| File | Contents |
|---|---|
| `scans.csv` | one row per step per scan |
| `scans.npz` | the same data as numpy arrays |
| `metadata.json` | sensor info (VV/PP/II), command used, timing, summary and error counts |
| `log.txt` | communication log |
| `raw.txt` | raw lidar replies, can be re-decoded with `--parse` |

### Main columns

| Column | Unit | Meaning |
|---|---|---|
| `scan_id` | | scan number (from 0) |
| `angle_deg` | ° | 0° = straight ahead, positive = left |
| `distance_mm` | mm | distance, `NaN` if the point is invalid |
| `valid` | 0/1 | 1 = usable point |
| `x_mm`, `y_mm` | mm | x positive to the right, y positive forward; origin = lidar centre |
| `raw_value` | | raw value incl. error code (values < 20 are error codes) |

Full column list and error codes: [code/lidar/README.md](../code/lidar/README.md).
Invalid points are kept (not removed or interpolated), only marked `valid = 0`.

## Loading the data

```python
import numpy as np
d = np.load("measurement_6_bricks/scans.npz")
ok = d["valid"] == 1
x, y = d["x_mm"][ok], d["y_mm"][ok]
```

To regenerate the scatter plot from raw data:

    python ../code/lidar/urg04lx.py --parse measurement_6_bricks/raw.txt --out measurement_6_bricks_plot --plot
