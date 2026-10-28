# Track A presentation build

The canonical GitHub project page is now [README.md](README.md), with the full
technical report under [docs/](docs/TEAM_SUBMISSION.md). These notes remain a
team operating reference. The supplied v2 guide additionally requires video
uncertainty evidence; the existing logs omit covariance histories, so that
display still needs to be captured before final submission.

## Platform support

This project must run on both macOS and Windows. Keep code, dependencies and file paths compatible with both platforms, and document separate launch commands where needed. Validate GUI changes on both platforms; macOS uses `mjpython`, while Windows uses the environment's Python directly.

## Project scope

This project is simulation-only. All sensing, estimation, navigation and docking run in MuJoCo. Physical robot implementation, real-sensor calibration, hardware induction and simulation-to-hardware comparison are outside the team scope.

The submission will demonstrate the complete simulated pipeline, including simulated camera calibration, state-space modelling and analysis, EKF estimation, control using the estimated state, and quantitative robustness testing under simulated noise and disturbances. Deliverables are the GitHub project page with code and results, a five-minute demo video, and each member's individual viva voce.

Hardware is optional under the assignment brief; removing it does not remove any compulsory simulation criterion. Any High Distinction claim must be supported by the independent contribution requirements in the marking criteria through simulation work.

## Run

### TurtleBot3 Waffle Pi (dynamic model)

The ROBOTIS model and required meshes are included under `vendor/robotis_tb3/`, with their license and provenance. No ROS installation or robot connection is needed. From the project directory:

To avoid opening an older build, use **Run Simulation.command** on macOS or
**Run Simulation.bat** on Windows. Both select `turtlebot.py` and the project
environment. Close any already-running simulation before relaunching: an open
Python process does not reload edited code. The terminal commands below do the
same thing.

```bash
# macOS
./.venv/bin/python .venv/bin/mjpython turtlebot.py
```

```powershell
# Windows
.\.venv\Scripts\python.exe turtlebot.py
```

This entry point builds `scene_turtlebot.xml` and runs wheel/contact physics at a 2 ms physics timestep. Wheel-joint rotations provide encoder measurements. The outer sensing/EKF/LQR pipeline runs at 50 Hz and uses the Waffle Pi's 0.288 m wheel separation. The camera mast, 72-ray 360-degree planar scan and gyro are project sensor additions. Wheel motors are torque-limited and tuned, and tyres use smooth cylinder collision proxies. These are simulation adaptations, not hardware-identified parameters. The log is `run_log_turtlebot.csv`, separate from the baseline log.

The dynamic model also fuses camera-derived heading from the known panel orientation, alongside range and bearing. The panel is assumed aligned with world -x and the camera has zero mounting yaw. A marker search near the target prevents odometry-only settling when the marker is lost. These assumptions must be updated if the station or camera mount is rotated.

Prediction uses encoder distance for translation and a simulated gyroscope for
heading increments, avoiding yaw drift from tyre slip during turns. The gyro
measures site angular velocity; it does not measure base orientation. Its model
assumes zero bias and 0.02 rad/s Gaussian white noise at the 50 Hz input, with
matched increment covariance. Bias and dropout robustness are not established.
The high-noise tests double this noise along with encoder and camera noise.

`navigation.py` constructs a temporary grid from current LiDAR returns transformed
with the EKF pose. A* plans around inflated returns; LQR tracks a committed local
waypoint. It does not read obstacle geometry from the simulator. Unknown space is
provisionally free and current-scan braking guards forward and reverse motion.
The enclosing 0.40 m footprint includes a clearance margin. Docking uses estimated
pose feedback, marker reacquisition and a reverse/reapproach manoeuvre to recover
lateral error. A controller stop uses an 0.08 m estimated position tolerance and
8 degree heading tolerance plus consecutive camera confirmation. The independent
trial pass criteria remain 0.15 m actual position error, 15 degrees actual heading
error and no observed hazard contacts.

Run `python scripts/validate_simulation.py --turtlebot` in the project environment for the offscreen model/camera/docking checks. Generate its report plots with `python scripts/plot_results.py run_log_turtlebot.csv`; outputs go to `output/results_turtlebot/`.

The earlier model's single validation run is retained in `output/turtlebot_validation/` for history; it is not evidence for the revised controller. An estimated-state stopping tolerance does not guarantee the same actual error. Windows execution remains unverified.

### Original simplified baseline

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe generate_marker.py
.\.venv\Scripts\python.exe main.py
```

On macOS, run from the project directory:

```bash
./.venv/bin/python generate_marker.py
./.venv/bin/python .venv/bin/mjpython main.py
```

Close the MuJoCo window to quit. On other platforms, **Q** in the OpenCV camera window also quits. The separate OpenCV window is disabled on macOS because it conflicts with the GUI thread used by `mjpython`; simulated camera rendering and ArUco processing remain active.

## What the system contains

- MuJoCo room, obstacles, robot, dock and ArUco target.
- MuJoCo-native 360-degree planar range sensing on TurtleBot3.
- Differential-drive command conversion from body velocity to left/right wheel angular speed.
- Noisy wheel-distance measurements used as simulated encoder odometry.
- EKF state estimate `[x_hat, y_hat, theta_hat]`.
- Gyro increments in prediction; camera ArUco range, bearing and heading correction.
- Discrete LQR controller operating on the EKF estimate, not ground-truth state.
- Scan-grid path planning, footprint braking and estimated-state docking recovery.
- CSV run log containing ground truth and EKF estimate for later quantitative evaluation.

## Robustness trials

Run the same dynamic controller without desktop windows:

```bash
# macOS
./.venv/bin/python scripts/run_robustness.py
```

```powershell
# Windows
.\.venv\Scripts\python.exe scripts/run_robustness.py
```

The default suite runs four conditions with seeds 7 and 19: baseline, randomised
known starting pose, doubled sensor noise with matched EKF covariance, and a
2 N lateral force for 0.3 simulated seconds. Each trial has a 140 simulated-second
limit. Every trial runs in a fresh process, saving its own CSV, JSON metrics and
terminal transcript under a new `output/robustness_<timestamp>/` directory.
`summary.csv`, `summary.md` and `config.json` record the outcomes and conditions;
`source/` retains the tested source files and their hashes.
Demo logs are preserved. A pass requires controller docking, actual target error
<=0.15 m, actual heading error <=15 degrees, and no observed robot contact with
obstacles, walls or the dock. Contacts are checked at every physics step. Ground
truth is used only to evaluate the result. Use `--seeds 7 19 31` for more repeats,
or `--cases baseline high_noise` to select conditions. These are exploratory
samples; report failures and limitations alongside successes.

The initial eight-trial macOS suite is saved under `output/robustness_initial/`.
Baseline and doubled-noise cases passed 2/2 each. Varied-start and push cases
passed 0/2 each. See `findings.md` there for failure details. Revised fixed-suite
results are stored separately under `output/robustness_final/`. The earlier
`robustness_revised/` run was superseded during debugging and is incomplete. Directories
ending in `_probe`, `navigation_debug` and `robustness_smoke*` contain development
checks, not uniform assessment comparisons.

The final fixed suite passed **8/8**, compared with **4/8** initially, with zero
observed hazard contacts. Actual docking errors were 0.056-0.082 m and completion
times were 62.9-89.5 simulated seconds. See
`output/robustness_comparison/comparison.md` and `comparison.png` for the complete
before/after report. The scanner and gyro changes mean this compares complete
pipelines rather than isolating a controller gain. This small, development-used
seed set does not establish performance on unseen cases. Windows, sensor bias,
dropouts and unknown initial localisation error remain unverified.

Use `python scripts/check_navigation.py` for fast geometry/recovery checks and
`python scripts/validate_simulation.py --turtlebot --camera-only` for offscreen
known-pose camera and Jacobian checks. `python scripts/compare_robustness.py`
regenerates the fixed-suite comparison after checking matching cases and criteria.

## Plot a saved run

After closing the simulation normally to save `run_log.csv`, run:

```bash
# macOS
./.venv/bin/python scripts/plot_results.py
```

```powershell
# Windows
.\.venv\Scripts\python.exe scripts/plot_results.py
```

Install the updated `requirements.txt` first if Matplotlib is missing. Figures, metrics and a short results summary are written to `output/results/`. The reported docking metrics stop at the first `DOCKED` sample; idle time afterwards is excluded. Save each trial's log before rerunning the simulation, which overwrites `run_log.csv`.

## Pipeline

The upright ArUco panel is labelled **CHARGING STATION** in the viewer. The status overlay shows navigation mode, marker detection/update acceptance, estimated distance to the docking target, and position error for evaluation only. A `DOCKED` stop requires consecutive camera-confirmed observations, position proximity and heading alignment. The target is deliberately offset from the panel to keep the complete marker in view.

Camera intrinsics come from the configured simulator resolution and field of view. Marker localisation uses the physical black-square size (excluding its white border), planar pose estimation, and the camera-to-robot offset. This is simulator-derived calibration; document it and the measurement validation results in the report.

At 640x480 pixels and 60 degree vertical FOV, `fx=fy=415.69 px` and the
principal point is `(320,240)`, with zero modelled lens distortion. The black
marker square is `0.68*700/980 = 0.4857 m`; the surrounding white margin is not
part of the PnP object square. Known-pose checks exercise range, bearing and
heading at three poses, with limits of 0.05 m, 0.02 rad and 0.15 rad respectively.
The camera measurement Jacobian is checked against finite differences.

| Measurement | Nominal added standard deviation | Frame / convention |
|---|---|---|
| Wheel travel | 0.0007 m per wheel per control step | Forward wheel rotation is positive |
| Camera horizontal range | 0.05 m | Camera origin, 0.18 m ahead of base |
| Camera bearing | 2 degrees | Positive left of robot forward |
| Camera-derived heading | 8 degrees | World yaw, panel orientation assumed known |
| Gyro yaw rate | 0.02 rad/s | IMU site +z, zero-bias assumption |

World +z is up; robot +x is forward and +y is left. OpenCV optical +z is
forward, +x is right and +y is down. The camera is at base-relative z=0.55 m;
the planar scan is at z=0.23 m. Gyro rates are integrated over the control period,
not interpreted as absolute yaw. EKF prediction propagates encoder and gyro
noise separately; the camera update uses range/bearing/heading innovation,
wrapped angular residuals, outlier gating and a Joseph covariance update.

For a repeatable offscreen check, run `python scripts/validate_simulation.py` using the project environment. It checks the camera Jacobian, rendered marker measurements at known poses and one complete navigation/docking trial. It writes a separate temporary log without replacing `run_log.csv`. The default seeded trial reached camera-confirmed docking with approximately 0.07 m actual target error on macOS. This single trial is a regression check; repeated robustness trials and Windows runtime verification are still required.

`wheel encoder distance + gyro yaw increment -> EKF prediction`

`camera ArUco range/bearing/heading -> EKF correction`

`EKF estimated state -> LQR -> v, omega`

`v, omega -> left/right wheel commands -> robot moves`

`360-degree range scan + EKF pose -> inflated grid -> local waypoint -> LQR`

## Important honesty for the tutor

`main.py` retains the original simplified differential-drive baseline. `turtlebot.py` uses the ROBOTIS TurtleBot3 Waffle Pi mesh, inertias, wheel joints and collision model, with project-specific simulated sensor mounting and motor-servo tuning. Ground truth is retained only for plant simulation and evaluation; navigation/control decisions use the EKF state estimate and sensor measurements. Baseline plots are not evidence for the dynamic model. Windows runtime verification and repeated robustness trials remain required.
