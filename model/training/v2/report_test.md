# Evaluation — v2\weights\best.pt

- split: `test`
- data: `..\dataset\export\cone-detector-nfjog-v2\data.yaml`
- device: `0`, imgsz 640

## Per-class metrics (test)

```
class       inst        P        R    mAP50  mAP50-95
blue         302    0.941    0.848    0.909     0.763
magenta       13    0.000    0.000    0.029     0.026
orange        67    0.850    0.687    0.799     0.553
red          178    0.798    0.837    0.870     0.725
yellow       231    0.888    0.825    0.911     0.742
mean                                            0.562
```

## What the average hides

- magenta: only 13 instances; its mAP moves a lot on a handful of boxes.
- magenta trails blue by 0.737 mAP50-95 (0.026 vs 0.763). The average is carrying it.
- magenta: recall 0.00 — roughly 100% of them are being missed. Downstream that is a gap in the corridor, not a wrong label.
- orange: recall 0.69 — roughly 31% of them are being missed. Downstream that is a gap in the corridor, not a wrong label.

## Confusion

- blue (302 true): 1 called red (0%); 31 missed entirely (10%)
- magenta (13 true): 9 called red (69%); 4 missed entirely (31%)
- orange (67 true): 10 called red (15%); 10 missed entirely (15%)
- red (178 true): 1 called orange (1%); 19 missed entirely (11%)
- yellow (231 true): 2 called orange (1%); 19 missed entirely (8%)
- WATCH orange -> red: 10 of 67 (15%) — a dead end read as a gate hands off to junction_exec at a wall — the worst confusion on the track
- WATCH red -> orange: 1 of 178 (1%) — a gate read as a dead end misses the junction entirely
- WATCH magenta -> red: 9 of 13 (69%) — the goal read as a gate — the car passes the finish
