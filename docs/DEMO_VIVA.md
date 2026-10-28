# Five-minute demonstration and viva guide

[Project home](../README.md) · [Results](RESULTS.md) · [Submission](TEAM_SUBMISSION.md)

The video has not yet been recorded. This is a recording outline, not a claim
that a video artifact exists. Add the actual viewable URL to the README and
submission record after recording.

## Suggested five-minute structure

| Time | Show | Explain |
|---|---|---|
| 0:00–0:25 | Project title, three members and room | Track A task; simulation-only scope; station and obstacles |
| 0:25–1:05 | Pipeline diagram and model | Pose state, wheel/body inputs, 3/3 local ranks and estimated-state constraint |
| 1:05–1:50 | Sensor/estimator diagram, calibration table, error plot | Camera geometry, frames, encoder distance, gyro yaw increment and EKF correction |
| 1:50–3:20 | Continuous current-build navigation and docking | LiDAR planning, LQR waypoints, marker acquisition and any recovery manoeuvre; finish with accepted marker and DOCKED |
| 3:20–4:25 | Before/after figure and trial table | Same pass criteria; 4/8 to 8/8; actual docking errors; no observed contacts; sensing and control both changed |
| 4:25–5:00 | Limitations, repository and contribution summary | Small seed set, known initial pose, idealised sensors, Windows status and equal one-third contributions |

The demonstration segment allows about 90 seconds because revised fixed trials
took 62.9–89.5 simulated seconds. Record a complete run first; slow rendering
may take longer in wall time. If footage is edited or accelerated, identify that
clearly rather than presenting accelerated playback as real-time performance.

**Required footage:** the supplied v2 guide asks for at least one varied/disturbed
trial and the estimate **with its uncertainty** alongside true state. Open
`Record Demo.command` on Mac or `Record Demo.bat` on Windows to start the push
demo. The launcher does not record the screen automatically. Record the live
orange 95% position ellipse and the estimated/true pose and marginal ±2σ overlay.
Close normally to save `run_log_demo.csv`; then generate a full state plot with
`python scripts/plot_results.py run_log_demo.csv` using the project interpreter.
The resulting `state_uncertainty.png` supplements the footage. Historical logs
without covariance still produce the older plots, with no invented uncertainty.

## Recording checklist

- Launch the current dynamic build with the project environment.
- Keep manual wheel-control sliders untouched during evidence runs.
- Show the viewer title, station, terminal ranks and final mode/acceptance.
- Use readable framing; the LiDAR rays are diagnostics and can visually obscure the robot.
- Preserve the final log at normal shutdown before another run overwrites it.
- Use the fixed-suite figures, with axes/units and condition labels visible.
- Include a varied/disturbed trial and covariance-based uncertainty display alongside estimate and truth.
- Distinguish actual target error, estimated target distance and estimation error.
- Confirm audio, duration and the video link's accessibility before submission.
- Include the AI tool/purpose declaration and the team's actual contribution statements on the linked project page.

Do not claim that every live run reproduces the exact fixed-suite numbers. Use
the saved logs for numerical statements and identify which trial a plot shows.
Keep the initial failed results available; they explain the improvements.

## Viva preparation

Each member should be able to explain the integrated pipeline, not only one
file. These are prompts for understanding, not a guaranteed list of viva questions.

| Question | Evidence to explain |
|---|---|
| What is the state and what are the inputs? | Planar pose, body velocities, wheel conversion; distinguish reduced model from contact plant. |
| Why rank 3/3, and is that always true? | Nonzero nominal speed couples yaw to lateral position; local rank at the selected operating point; stopped linearisation differs. |
| Why is range/bearing alone insufficient in one frame? | Two measurements for three states; motion and camera-derived heading add constraints. |
| What exactly was calibrated? | Simulator FOV/intrinsics, black-square geometry, camera mount/frame signs and known-pose validation; no real-sensor calibration. |
| Why use the gyro? | Wheel slip corrupted heading during turns; gyro supplies rate increments, not absolute ground-truth orientation. |
| What happens when the camera loses the marker? | Encoder/gyro prediction continues; acquired docking can stop translation and search/reacquire. |
| How do Q and R differ? | Process uncertainty versus measurement uncertainty; units/variances; matched high-noise experiment. |
| What does the innovation gate do? | Detect inconsistent measurements using innovation covariance; detection can be rejected. |
| Is all control LQR? | LQR tracks local waypoints; docking/search/realignment use separate estimated-state feedback and mode logic. |
| How are collisions avoided and checked? | Inflated scan grid, guarded commands, independent every-physics-step contact monitoring. |
| Why can DOCKED still have nonzero actual error? | Stop uses noisy estimated state; evaluation uses actual pose separately. |
| What do the results prove? | Eight defined development-used cases passed; the corrected Windows build also passed eight defined cases after rejecting invalid camera data. These development-used samples do not establish reliability in all environments or under bias/dropouts. |
| What did you personally contribute? | Equal one-third contribution to Project 17; explain your work and how it connects to the integrated pipeline. |

Recommended preparation order: [Model](MODEL.md), [Sensing](SENSING.md),
[Estimation](ESTIMATION.md), [Control](CONTROL.md), then [Results](RESULTS.md).
Be ready to locate the corresponding functions and explain a failed initial run.
