# Data

- `layouts/` — surveyed cone positions + ground-truth centerlines (D5).
  One file per layout. Convention (fix BEFORE the first measurement): fixed
  origin, measure to cone **base centers**, x/y axes agreed, one shared sheet.
- `trials/` — per-run logs and the trial summary CSV (D6). Rosbags themselves
  are gitignored (too large); keep extracted CSVs and per-run notes here.
