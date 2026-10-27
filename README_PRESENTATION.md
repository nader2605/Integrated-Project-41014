# Track A presentation build

## Run

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe generate_marker.py
.\.venv\Scripts\python.exe main.py
```

Press **Q** in the OpenCV camera window to quit.

## What the system contains

- MuJoCo room, obstacles, robot, dock and ArUco target.
- MuJoCo-native rangefinder fan for obstacle sensing.
- Differential-drive command conversion from body velocity to left/right wheel angular speed.
- Noisy wheel-distance measurements used as simulated encoder odometry.
- EKF state estimate `[x_hat, y_hat, theta_hat]`.
- Camera ArUco detection providing range + bearing correction to the EKF.
- Discrete LQR controller operating on the EKF estimate, not ground-truth state.
- LiDAR avoidance override when an obstacle blocks the path.
- CSV run log containing ground truth and EKF estimate for later quantitative evaluation.

## Presentation chain

`wheel encoders -> EKF prediction`

`camera ArUco range/bearing -> EKF correction`

`EKF estimated state -> LQR -> v, omega`

`v, omega -> left/right wheel commands -> robot moves`

`native range sensors -> obstacle detected -> temporary avoidance -> return to LQR`

## Important honesty for the tutor

The current robot is a simplified differential-drive mobile-robot plant rendered in MuJoCo. It follows the Track A architecture, but it is not yet a full TurtleBot3 URDF/dynamic wheel-contact model. Ground truth is retained only for rendering and evaluation; navigation/control decisions are based on the EKF state estimate and sensor measurements.
