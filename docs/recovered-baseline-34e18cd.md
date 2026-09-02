# Recovered on-car baseline (`34e18cd`)

This commit preserves the runtime source that was deployed and working on
`ucsdrobocar-148-02` on 2026-09-01. The deployed `VERSION` file identified its
source revision as `34e18cd`, but that Git object was not available in the local
clone and neither the development Mac nor the Pi could authenticate to the
private GitHub repository during recovery.

The following deployed paths were copied back into their source-tree locations:

- `~/cone_capture_tool/` to `model/capture/`
- `~/cone_capture_tool/cone_nav/` to `ros2/src/cone_nav/cone_nav/`
- `~/cone_capture_tool/cone_perception/` to
  `ros2/src/cone_perception/cone_perception/`
- `~/cone_capture_tool/routes/` to `data/routes/`
- `~/mycar/myconfig_capture.py` to `model/capture/myconfig_capture.py`

An rsync checksum dry run verified that the recovered runtime files matched the
Pi. Runtime output and machine-specific state were deliberately excluded:
trial JSONL/MCAP files, sessions, caches, `calibration.json`, the generated
`VERSION` file, and the manually uploaded audio directory. Tests and other
source-only files that were excluded by the original deployment process could
not be recovered from the Pi.
