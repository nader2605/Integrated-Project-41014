# Simulation run results

Source: `push_seed7.csv`

| Metric | Value |
|---|---|
| result | DOCKED |
| wall_time_to_first_docked_s | 71.8153 |
| actual_target_error_m | 0.0642 |
| estimated_target_error_m | 0.0664 |
| actual_heading_error_deg | 2.9300 |
| position_rmse_through_docking_m | 0.0551 |
| maximum_position_error_through_docking_m | 0.1833 |
| heading_rmse_through_docking_deg | 1.0913 |
| aruco_detection_fraction_through_docking | 0.4080 |
| actual_path_length_through_docking_m | 5.9160 |

Metrics use samples through the first DOCKED state, excluding subsequent idle time. The error plot also shows the idle period. Times are logged wall-clock times, not simulated physics time. Actual target error and EKF position error are distinct quantities. Ground truth is used here only for evaluation.

This is one seeded run, not a success-rate or robustness study. It does not establish collision-free motion. True pose, estimate and covariance are time-aligned before the current movement.

state_uncertainty.png shows marginal +/-2 sigma (approximately 95.45% for a calibrated Gaussian). Trajectory ellipses use the joint 2D 95% chi-square factor, not 2 sigma. These are model uncertainty estimates, not guaranteed error bounds or proof of filter consistency.
