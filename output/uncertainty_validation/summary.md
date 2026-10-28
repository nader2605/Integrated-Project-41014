# TurtleBot3 robustness results

Success requires camera-confirmed controller docking, actual position error <=15 cm, heading error <=15 degrees and no robot contact with obstacles, walls or dock.

| Condition | Completed trials | Successes | Success rate |
|---|---:|---:|---:|
| push | 1 | 1 | 100% |

Elapsed/completion times in summary.csv are simulated seconds, excluding settling. Per-step logs retain wall-clock time. Contact is checked every physics step. These small samples are exploratory evidence, not a reliable population success-rate estimate. Noise covariance is adjusted for the high-noise case; unknown-noise mismatch is not tested. Initial pose is known; initial localisation error is not tested. Windows requires a separate runtime check. See config.json for conditions.
