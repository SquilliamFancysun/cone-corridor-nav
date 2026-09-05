# Evaluation — v1/weights/best.pt

- split: `test`
- data: `dataset/export/cone-detector-nfjog-v1/data.yaml`
- device: `mps`, imgsz 640

## Per-class metrics (test)

```
class       inst        P        R    mAP50  mAP50-95
blue          95    0.964    0.758    0.778     0.648
magenta       13    0.000    0.000    0.000     0.000
orange        16    0.780    0.625    0.814     0.519
red           37    0.772    0.825    0.856     0.729
yellow        61    0.799    0.787    0.819     0.674
mean                                            0.514
```

## What the average hides

- magenta: only 13 instances; its mAP moves a lot on a handful of boxes.
- orange: only 16 instances; its mAP moves a lot on a handful of boxes.
- magenta trails red by 0.729 mAP50-95 (0.000 vs 0.729). The average is carrying it.
- magenta: recall 0.00 — roughly 100% of them are being missed. Downstream that is a gap in the corridor, not a wrong label.
- orange: recall 0.62 — roughly 38% of them are being missed. Downstream that is a gap in the corridor, not a wrong label.

## Confusion

- blue (95 true): 22 missed entirely (23%)
- magenta (13 true): 9 called red (69%); 4 missed entirely (31%)
- orange (16 true): 1 called red (6%); 3 missed entirely (19%)
- red (37 true): 1 called orange (3%); 2 missed entirely (5%)
- yellow (61 true): 1 called orange (2%); 10 missed entirely (16%)
- WATCH orange -> red: 1 of 16 (6%) — a dead end read as a gate hands off to junction_exec at a wall — the worst confusion on the track
- WATCH red -> orange: 1 of 37 (3%) — a gate read as a dead end misses the junction entirely
- WATCH magenta -> red: 9 of 13 (69%) — the goal read as a gate — the car passes the finish
