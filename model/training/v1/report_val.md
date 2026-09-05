# Evaluation — v1/weights/best.pt

- split: `val`
- data: `/Users/home/Projects/MAE 148/cone-corridor-nav/model/dataset/export/cone-detector-nfjog-v1/data.yaml`
- device: `mps`, imgsz 640

## Per-class metrics (val)

```
class       inst        P        R    mAP50  mAP50-95
blue          59    0.949    0.797    0.940     0.761
magenta        7    0.000    0.000    0.000     0.000
orange        17    0.983    0.765    0.899     0.518
red           28    0.710    0.893    0.818     0.667
yellow        44    0.871    0.909    0.914     0.679
mean                                            0.525
```

## What the average hides

- magenta: only 7 instances; its mAP moves a lot on a handful of boxes.
- orange: only 17 instances; its mAP moves a lot on a handful of boxes.
- red: only 28 instances; its mAP moves a lot on a handful of boxes.
- magenta trails blue by 0.761 mAP50-95 (0.000 vs 0.761). The average is carrying it.
- magenta: recall 0.00 — roughly 100% of them are being missed. Downstream that is a gap in the corridor, not a wrong label.

## Confusion

- blue (59 true): 5 missed entirely (8%)
- magenta (7 true): 5 called red (71%); 2 missed entirely (29%)
- orange (17 true): 2 called red (12%); 2 missed entirely (12%)
- yellow (44 true): 2 missed entirely (5%)
- WATCH orange -> red: 2 of 17 (12%) — a dead end read as a gate hands off to junction_exec at a wall — the worst confusion on the track
- WATCH magenta -> red: 5 of 7 (71%) — the goal read as a gate — the car passes the finish
