# Simulation run results

Source: `start_pose_seed7.csv`

| Metric | Value |
|---|---|
| result | No DOCKED state recorded |
| wall_time_to_first_docked_s | None |
| actual_target_error_m | 1.4837 |
| estimated_target_error_m | 0.6001 |
| actual_heading_error_deg | 82.0960 |
| position_rmse_through_docking_m | 0.5629 |
| maximum_position_error_through_docking_m | 0.9204 |
| heading_rmse_through_docking_deg | 39.9293 |
| aruco_detection_fraction_through_docking | 0.0687 |
| actual_path_length_through_docking_m | 2.9372 |

Metrics use the whole saved run; no DOCKED state was recorded. The error plot also shows the idle period. Times are logged wall-clock times, not simulated physics time. Actual target error and EKF position error are distinct quantities. Ground truth is used here only for evaluation.

This is one seeded run, not a success-rate or robustness study. It does not establish collision-free motion. The log compares the estimate before the current movement with the plant after that movement, so instantaneous estimation errors include one control-step offset.
