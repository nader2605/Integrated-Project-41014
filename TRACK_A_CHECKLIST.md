# Track A checklist

Scope: simulation-only. Hardware implementation and induction are excluded.

- [x] MuJoCo simulation
- [x] Mobile differential-drive robot
- [x] Obstacle room
- [x] Native simulated range sensing
- [x] Camera
- [x] ArUco detection
- [x] Wheel-encoder odometry simulation
- [x] EKF fusing encoder prediction + camera range/bearing update
- [x] State-space linearisation used for LQR
- [x] Controllability rank check
- [x] Observability rank check
- [x] LQR controller uses `x_hat`, not ground truth
- [x] LiDAR obstacle avoidance
- [x] Visual docking stage
- [x] Ground-truth vs estimate logging for evaluation
- [ ] Validate/tune this exact build on the user's machine
- [x] Add ROBOTIS TurtleBot3 Waffle Pi with dynamic wheel/contact physics (`turtlebot.py`)
- [ ] Validate dynamic TurtleBot3 on Windows
- [x] Run repeated/randomised trials and plot quantitative results (`output/robustness_comparison/`)
- [x] Run initial eight-trial robustness suite; preserve all successes and failures (`output/robustness_initial/`)
- [x] Improve varied-start obstacle clearance and post-disturbance docking; rerun the fixed eight-trial suite (8/8 passed)
- [x] Document simulator-derived camera calibration, frames, noise and known-pose checks
- [x] Evaluate the specified start, sensor-noise and push conditions; retain limited-sample caveats
- [ ] Publish GitHub project page with code, model/analysis and quantitative results
- [x] Draft GitHub README and technical documentation for Project 17
- [x] Add the required generative-AI tools/purpose declaration to the project page
- [ ] Confirm each member's individual contribution statement
- [ ] Capture estimate uncertainty alongside estimated/true state for the video (required by v2 guide; current CSVs omit covariance)
- [ ] Record five-minute simulation demo video
- [ ] Prepare each member for the individual viva voce
