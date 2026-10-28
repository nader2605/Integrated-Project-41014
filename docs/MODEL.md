# Platform model and analysis

[Project home](../README.md) · [Estimator](ESTIMATION.md) · [Controller](CONTROL.md)

## Physical plant versus control model

`turtlebot.py` loads an adapted ROBOTIS TurtleBot3 Waffle Pi model. Its wheel
velocity actuators, joint rotations, chassis inertias and contacts are integrated
by MuJoCo at 0.002 s. The outer algorithm runs at 0.02 s. Commands are applied to
wheel actuators; the dynamic run does not overwrite base pose on every control
step. Pose resets occur only at initialization and in known-pose validation.

The controller uses a reduced planar **kinematic** state-space model. It does
not identify all rigid-body or wheel-servo dynamics. Contact slip, torque
saturation, rolling geometry and tracking lag explain differences between this
model and the simulated plant.

| Parameter | Dynamic configuration |
|---|---:|
| Wheel radius, r | 0.033 m |
| Wheel separation, b | 0.288 m |
| Wheel speed limit | ±7.88 rad/s |
| Wheel torque limit | ±0.2 N·m |
| Outer sample time, T | 0.02 s |
| Physics timestep | 0.002 s |
| Nominal spawn | (−2.05 m, −1.35 m, 0 rad) |
| Pre-dock target | (0.95 m, 0 m) |
| Docking base target | (1.55 m, 0 m, 0 rad) |

The station body is at x=2.45 m; the visible marker plane is at x=2.391 m. The
base target is deliberately offset from the panel so the marker stays visible.
No battery charging or electrical docking is simulated.

## Nonlinear model

Define pose and body-velocity input:

$$
\mathbf{x}=[p_x,p_y,\theta]^T,\qquad \mathbf{u}=[v,\omega]^T.
$$

Under planar, differential-drive and ideal rolling assumptions:

$$
\dot{\mathbf{x}}=
\begin{bmatrix}v\cos\theta\\v\sin\theta\\\omega\end{bmatrix}.
$$

Wheel/body conversions are

$$
v=\frac{r}{2}(\omega_R+\omega_L),\quad
\omega=\frac{r}{b}(\omega_R-\omega_L),\quad
\omega_{L,R}=\frac{v\mp b\omega/2}{r}.
$$

Wheel commands are individually clipped. Consequently a saturated command need
not preserve the requested curvature. The gyro is used in the final estimator
because actual yaw can differ substantially from wheel-difference odometry
during contact-rich turns.

## Linearisation used by LQR

For small deviations about heading θ₀ and nonzero nominal forward speed v₀:

$$
\delta\dot{\mathbf{x}}=A_c\delta\mathbf{x}+B_c\delta\mathbf{u},\quad
A_c=\begin{bmatrix}0&0&-v_0\sin\theta_0\\0&0&v_0\cos\theta_0\\0&0&0\end{bmatrix},\quad
B_c=\begin{bmatrix}\cos\theta_0&0\\\sin\theta_0&0\\0&1\end{bmatrix}.
$$

`linearised_model()` uses forward-Euler discretisation:

$$A_d=I+TA_c,\qquad B_d=TB_c.$$

The gain is recomputed using estimated heading and a local nominal speed. This
is a sequence of local approximations, not a global linear model of the robot.

## Controllability

The implementation evaluates

$$\mathcal C=[B_d\; A_dB_d\; A_d^2B_d].$$

At θ₀=0, v₀=0.20 m/s, T=0.02 s:

$$
A_d=\begin{bmatrix}1&0&0\\0&1&0.004\\0&0&1\end{bmatrix},\qquad
B_d=\begin{bmatrix}0.02&0\\0&0\\0&0.02\end{bmatrix}.
$$

The recorded rank is **3/3**. Forward motion couples a heading change into
lateral displacement. At v₀=0 the same instantaneous linearisation has rank 2;
it cannot represent sideways motion at rest. This does not imply the nonlinear
unicycle can never reach a different lateral position. It explains the use of
nonzero-speed local tracking and a manoeuvre-based docking controller.

## Observability

The camera range/bearing model and Jacobian H are derived in
[Estimation](ESTIMATION.md). `analyse_model()` forms

$$\mathcal O=[H^T\;(HA_d)^T\;(HA_d^2)^T]^T.$$

At the code's selected state (−1.0, −0.6, 0), with the known panel location and
the same A_d, the recorded rank is **3/3**. This is a local, visible-landmark,
linearised analysis. It is not a claim of global observability throughout an
occluded run. Without camera corrections, gyro/encoder integration can drift.

The dynamic estimator additionally observes camera-derived heading from the
known panel orientation. Appending `[0,0,1]` to H gives local full pose information
for nonzero landmark distance. A single range/bearing observation alone has at
most rank 2; motion and/or heading information provide the additional constraint.

## Assumptions and evidence

- The planar controller ignores small roll/pitch and height changes in contact dynamics.
- The initial pose and panel location/orientation are known; unknown initial
  localisation is not part of the saved suite.
- Sensor mounting and wheel dimensions are specified simulation parameters,
  not hardware measurements.
- The printed ranks describe the reduced model at the stated operating points.
  They do not prove robustness, collision avoidance or global stability.

Implementation: `main.py` functions `linearised_model`, `analyse_model`,
`body_to_wheels`; physical adapter: `turtlebot.py`; generated model builder:
`scripts/build_turtlebot_scene.py`. See [references](REFERENCES.md) for provenance.
