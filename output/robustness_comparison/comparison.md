# Robustness comparison

Same cases, seeds 7, 19, 140 simulated-second limits and evaluation criteria. The revised pipeline adds denser 360-degree sensing and gyro prediction as well as control changes; the comparison does not isolate the contribution of any one change.

| Condition | Initial passes | Revised passes |
|---|---:|---:|
| baseline | 2/2 | 2/2 |
| start_pose | 0/2 | 2/2 |
| high_noise | 2/2 | 2/2 |
| push | 0/2 | 2/2 |

| Trial | Initial final error (m) | Revised final error (m) | Revised result |
|---|---:|---:|---|
| baseline / 7 | 0.130 | 0.068 | PASS |
| baseline / 19 | 0.057 | 0.065 | PASS |
| start_pose / 7 | 1.484 | 0.077 | PASS |
| start_pose / 19 | 1.533 | 0.072 | PASS |
| high_noise / 7 | 0.126 | 0.056 | PASS |
| high_noise / 19 | 0.108 | 0.082 | PASS |
| push / 7 | 0.406 | 0.064 | PASS |
| push / 19 | 1.495 | 0.071 | PASS |

Revised successful trials: final position error 0.056-0.082 m; completion 62.9-89.5 simulated seconds.

A final error for a timed-out trial is not a docking error. Success requires controller docking, actual target error <=0.15 m, actual heading error <=15 degrees and zero observed hazard contact. The initial results are retained unchanged. Pilot/adaptive runs are excluded. Two seeds per condition are exploratory evidence, not a population success-rate estimate. Sensor bias/dropout, unknown initial localisation error and Windows runtime remain unverified.
