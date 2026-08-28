# Evaluation — v1/weights/best.pt

- split: `val`
- data: `dataset/export/cone-detector-nfjog-v1/data.yaml`
- device: `mps`, imgsz 640

## Per-class metrics (val)

```
class       inst        P        R    mAP50  mAP50-95
blue          59    0.903    0.789    0.916     0.700
magenta        7    0.000    0.000    0.000     0.000
orange        17    0.732    0.588    0.663     0.398
red           28    0.764    0.923    0.898     0.698
yellow        44    0.825    0.886    0.878     0.657
mean                                            0.490
```

## What the average hides

- magenta: only 7 instances; its mAP moves a lot on a handful of boxes.
- orange: only 17 instances; its mAP moves a lot on a handful of boxes.
- red: only 28 instances; its mAP moves a lot on a handful of boxes.
- magenta trails blue by 0.700 mAP50-95 (0.000 vs 0.700). The average is carrying it.
- magenta: recall 0.00 — roughly 100% of them are being missed. Downstream that is a gap in the corridor, not a wrong label.
- orange: recall 0.59 — roughly 41% of them are being missed. Downstream that is a gap in the corridor, not a wrong label.

## Confusion

- blue (59 true): 8 missed entirely (14%)
- magenta (7 true): 3 called red (43%); 4 missed entirely (57%)
- orange (17 true): 2 called red (12%); 1 missed entirely (6%)
- yellow (44 true): 4 missed entirely (9%)
- WATCH orange -> red: 2 of 17 (12%) — a dead end read as a gate hands off to junction_exec at a wall — the worst confusion on the track
- WATCH magenta -> red: 3 of 7 (43%) — the goal read as a gate — the car passes the finish
