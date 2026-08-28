# Evaluation — v1/weights/best.pt

- split: `test`
- data: `dataset/export/cone-detector-nfjog-v1/data.yaml`
- device: `mps`, imgsz 640

## Per-class metrics (test)

```
class       inst        P        R    mAP50  mAP50-95
blue          34    1.000    0.591    0.615     0.538
magenta        8    0.000    0.000    0.000     0.000
orange         4    0.947    0.500    0.711     0.597
red            4    0.447    0.500    0.578     0.570
yellow        29    1.000    0.689    0.773     0.650
mean                                            0.471
```

## What the average hides

- magenta: only 8 instances; its mAP moves a lot on a handful of boxes.
- orange: only 4 instances; its mAP moves a lot on a handful of boxes.
- red: only 4 instances; its mAP moves a lot on a handful of boxes.
- yellow: only 29 instances; its mAP moves a lot on a handful of boxes.
- magenta trails yellow by 0.650 mAP50-95 (0.000 vs 0.650). The average is carrying it.
- blue: recall 0.59 — roughly 41% of them are being missed. Downstream that is a gap in the corridor, not a wrong label.
- magenta: recall 0.00 — roughly 100% of them are being missed. Downstream that is a gap in the corridor, not a wrong label.
- orange: recall 0.50 — roughly 50% of them are being missed. Downstream that is a gap in the corridor, not a wrong label.
- red: recall 0.50 — roughly 50% of them are being missed. Downstream that is a gap in the corridor, not a wrong label.
- yellow: recall 0.69 — roughly 31% of them are being missed. Downstream that is a gap in the corridor, not a wrong label.

## Confusion

- blue (34 true): 13 missed entirely (38%)
- magenta (8 true): 5 called red (62%); 3 missed entirely (38%)
- orange (4 true): 2 called red (50%)
- red (4 true): 1 missed entirely (25%)
- yellow (29 true): 8 missed entirely (28%)
- WATCH orange -> red: 2 of 4 (50%) — a dead end read as a gate hands off to junction_exec at a wall — the worst confusion on the track
- WATCH magenta -> red: 5 of 8 (62%) — the goal read as a gate — the car passes the finish
