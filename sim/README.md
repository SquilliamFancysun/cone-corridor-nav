# Simulation & Replay

No hardware required — this is what lets planning/analysis work proceed while
the car is broken.

- **Synthetic cone-field generator**: emits labeled cone lists (same shape as
  `cone_msgs/LabeledConeArray`) for parameterized layouts, with configurable
  noise, dropout, and occlusion. The unit tests for `cone_nav` consume these.
- **Replay harness**: feeds recorded on-car logs through the pure-Python
  `cone_nav` layers offline, so a bug can be reproduced from a bag instead of
  a track session.
