# Simulation run results

Source: `start_pose_seed7.csv`

| Metric | Value |
|---|---|
| result | DOCKED |
| wall_time_to_first_docked_s | 51.6417 |
| actual_target_error_m | 0.0769 |
| estimated_target_error_m | 0.0739 |
| actual_heading_error_deg | 3.2144 |
| position_rmse_through_docking_m | 0.0392 |
| maximum_position_error_through_docking_m | 0.0962 |
| heading_rmse_through_docking_deg | 0.7430 |
| aruco_detection_fraction_through_docking | 0.6006 |
| actual_path_length_through_docking_m | 5.3251 |

Metrics use samples through the first DOCKED state, excluding subsequent idle time. The error plot also shows the idle period. Times are logged wall-clock times, not simulated physics time. Actual target error and EKF position error are distinct quantities. Ground truth is used here only for evaluation.

This is one seeded run, not a success-rate or robustness study. It does not establish collision-free motion. The log compares the estimate before the current movement with the plant after that movement, so instantaneous estimation errors include one control-step offset.
