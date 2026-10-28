# Reproduce and inspect results

[Project home](../README.md) · [Setup](SETUP.md) · [Results](RESULTS.md)

## Use the correct interpreter

Run commands from the repository root. In the examples below `PYTHON` means
`./.venv/bin/python` on macOS or `.\.venv\Scripts\python.exe` on Windows.
Replace the word with the executable path; it is not a shell command installed
by this repository. Offscreen checks do not launch the desktop viewer, but still
require a working graphics context for camera rendering.

## Quick checks

```bash
# macOS
./.venv/bin/python scripts/check_navigation.py
./.venv/bin/python scripts/validate_simulation.py --turtlebot --camera-only
```

```powershell
# Windows; navigation and camera checks passed
.\.venv\Scripts\python.exe scripts/check_navigation.py
.\.venv\Scripts\python.exe scripts/validate_simulation.py --turtlebot --camera-only
```

The geometry checks cover forward/reverse footprint braking, a path around a
barrier, acquisition latching and docking recovery. Camera checks cover three
rendered known poses, bearing/heading conventions, the measurement Jacobian,
and rejection of non-finite camera poses and EKF measurements.
Running validation without `--camera-only` also executes a single docking
regression trial and writes a temporary validation log. That script asserts
controller docking and position error <0.15 m; the full suite adds heading and
contact criteria and should be used for robustness claims.

## Run a fresh suite

```bash
./.venv/bin/python scripts/run_robustness.py
```

```powershell
.\.venv\Scripts\python.exe scripts/run_robustness.py
```

Default: four cases × seeds 7 and 19, 140 simulated seconds per trial. New output
uses a timestamped directory. Keep existing directories intact. Example options:

```bash
./.venv/bin/python scripts/run_robustness.py --cases baseline push --seeds 7 19
./.venv/bin/python scripts/run_robustness.py --seeds 31 43 --output output/robustness_holdout
```

The second command is a suggested future evaluation, not a completed result.
An explicit output directory must not already exist. Failed or timed-out trials
are retained and counted; do not omit them when calculating a success fraction.
Each trial has its own Python process to reset filter, random generator and
controller state. The runner continues after an execution error and records a
failure; inspect both its summary and transcripts rather than relying only on
the process exit code.

## Data files

| Output | Meaning |
|---|---|
| `config.json` | Cases, seeds, timing, condition definitions, platform, package versions, source hashes |
| `source/` | Copies of the listed tested Python sources |
| `<case>_seedN.csv` | Per-control-step pose, detection and mode log |
| `<case>_seedN.json` | Actual errors, docking flag, pass flag, simulated duration, contacts and force-step count |
| `<case>_seedN.txt` | Terminal output, progress, ranks and any traceback |
| `summary.csv` | One row per trial, including failures |
| `summary.md` | Counts and condition caveats |

The per-step CSV has:

```text
time_s,true_x,true_y,true_theta,est_x,est_y,est_theta,front_range_m,aruco_detected,mode
```

Positions/ranges are metres, yaw is radians and `time_s` is elapsed **wall-clock**
time. `aruco_detected` records image detection, not EKF acceptance. Acceptance is
shown in terminal/viewer status but is not a separate per-step CSV column.
The JSON's simulated duration comes from MuJoCo time after settling. With the
current fixed loop it equals `samples * 0.02` to floating-point tolerance.

Historical eight-trial logs do not include covariance. New logs add `sim_time_s`
and the six independent posterior covariance entries `P_xx`, `P_xy`, `P_xtheta`,
`P_yy`, `P_ytheta`, `P_thetatheta` before the range/detection/mode fields. True pose,
estimate and covariance are sampled together before motion. Old logs remain
supported; their uncertainty cannot be reconstructed from error data alone.

Neither format includes innovation histories or a desired trajectory time
series. Therefore saved pose RMSE is an estimation metric, and final target
error is a docking metric; neither is a complete time-indexed trajectory-tracking
RMSE. Do not label them interchangeably in the report.

## Plot a saved log

```bash
./.venv/bin/python scripts/plot_results.py run_log_turtlebot.csv
./.venv/bin/python scripts/plot_results.py output/robustness_final/start_pose_seed7.csv
```

Outputs include trajectory and error PNGs, metrics CSV and a Markdown summary.
Suite-log plots are placed under that suite's `plots/<trial>/`, preserving older
figures. `--output path/to/folder` explicitly selects a destination. A run without
`DOCKED` is labelled accordingly; its final error must not be called docking error.

For new covariance logs, plotting also creates `state_uncertainty.png` with
estimated and true x/y/yaw and marginal ±2σ bands. Trajectory plots add joint 2D
95% position ellipses. These are distinct probability conventions and rely on the
filter's covariance model, not guaranteed error bounds. Use
`python scripts/validate_demo.py` for an offscreen push-demo, covariance sanity
check and viewer-graphics preview; it writes `output/uncertainty_demo_validation/`.

Interactive `run_log_turtlebot.csv` is overwritten at the next normal shutdown.
The suite never uses that file as trial output. Preserve a demonstration log
before rerunning the interactive simulation.

## Regenerate the published comparison

```bash
./.venv/bin/python scripts/compare_robustness.py
```

This reads the initial and final fixed suites. It rejects incomplete/duplicated
suites and mismatched cases, seeds, time limit or success definition. It writes
`output/robustness_comparison/`. Use `--before`, `--after` and `--output` for other
completed suites. Do not compare a pilot run as though it were a fixed suite.

## Configuration integrity

The saved source hashes identify `main.py`, `turtlebot.py`, `navigation.py`,
the builder, runner and headless adapter used for the final suite. These matched
the working files at the completion of testing. The supplied robot assets are
identified separately by the upstream commit in `vendor/robotis_tb3/SOURCE.md`.

`requirements.txt` is not a lockfile. The recorded package versions help reproduce
the environment, but Python/OS, solver/library versions and floating-point
differences can alter trajectories. Rebuilding the same seed on another platform
requires a new validation record, not an assumption of bitwise-identical results.
