# Evaluation — v3\weights\best.pt

- split: `test`
- data: `C:\Dev\GitHub\cone-corridor-nav\model\dataset\export\cone-detector-nfjog-v3\data.yaml`
- device: `0`, imgsz 640

## Per-class metrics (test)

```
class       inst        P        R    mAP50  mAP50-95
blue         772    0.981    0.854    0.945     0.786
magenta      233    0.988    0.684    0.867     0.653
orange        67    0.885    0.687    0.783     0.583
red          178    0.796    0.835    0.856     0.714
yellow       765    0.962    0.925    0.972     0.836
mean                                            0.715
```

## What the average hides

- orange trails yellow by 0.253 mAP50-95 (0.583 vs 0.836). The average is carrying it.
- magenta: recall 0.68 — roughly 32% of them are being missed. Downstream that is a gap in the corridor, not a wrong label.
- orange: recall 0.69 — roughly 31% of them are being missed. Downstream that is a gap in the corridor, not a wrong label.

## Confusion

- blue (772 true): 2 called red (0%); 49 missed entirely (6%)
- magenta (233 true): 35 called red (15%); 20 missed entirely (9%)
- orange (67 true): 10 called red (15%); 9 missed entirely (13%)
- red (178 true): 2 called orange (1%); 16 missed entirely (9%)
- yellow (765 true): 6 called orange (1%); 16 missed entirely (2%)
- WATCH orange -> red: 10 of 67 (15%) — a dead end read as a gate hands off to junction_exec at a wall — the worst confusion on the track
- WATCH red -> orange: 2 of 178 (1%) — a gate read as a dead end misses the junction entirely
- WATCH magenta -> red: 35 of 233 (15%) — the goal read as a gate — the car passes the finish
