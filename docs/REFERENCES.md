# References, provenance and licensing

[Project home](../README.md)

## Supplied model

The TurtleBot3 Waffle Pi model and four referenced STL meshes come from
[ROBOTIS MuJoCo Menagerie — TurtleBot3](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie/tree/main/robotis_tb3).
The local source archive identifies upstream commit
`d8344c0dbe7a00208d0301111523dde65efc174a`.

The original XML, meshes and Apache-2.0 license are retained under
`vendor/robotis_tb3/`. See [SOURCE.md](../vendor/robotis_tb3/SOURCE.md) and the
[upstream license copy](../vendor/robotis_tb3/LICENSE). The generated project
scene carries an adaptation notice; original vendored assets remain unmodified.

Project adaptations include the room/station, elevated camera, planar scanner,
gyro, timestep, wheel-servo settings and smooth tyre collision proxies. These
must not be described as an unmodified stock simulation or a hardware-identified
model.

## Software references

- [MuJoCo Python bindings and viewer](https://mujoco.readthedocs.io/en/stable/python.html) — simulation API and platform-specific viewer operation.
- [MuJoCo XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html) — model, actuators, sites and sensors.
- [OpenCV ArUco detection](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) — marker concepts and detection workflow; installed API/version is recorded separately.
- [OpenCV camera calibration and pose estimation](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html) — intrinsic/pose geometry and `solvePnP`.
- [SciPy discrete algebraic Riccati solver](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_discrete_are.html) — solver used for local LQR gains.
- [GitHub Pages documentation](https://docs.github.com/en/pages) — optional website hosting, separate from repository Markdown rendering.

These references explain tools and concepts. The equations, parameters and
reported results in this repository are tied to the local implementation and
saved experiment evidence. Current online docs can differ from installed package
versions, so reproduce using the recorded configuration.

## Course marking guide

The team-provided **Sensors_Control_Integrated_Project v2.pdf** (five-page marking
guide) was used to check track scope, deliverables, the five assessment criteria,
individual viva weighting and the AI declaration requirement. It is distributed
through the course Canvas assignment. The PDF is not copied into this public
repository; consult the current course copy for authoritative requirements.

## Course-supplied connection example

The course also supplied
[UTS-RI/turtlebot_rosbridge](https://github.com/UTS-RI/turtlebot_rosbridge).
Its README describes a Python interface to a physical ROS robot over rosbridge.
That connection layer is not used or required by this simulation-only project.

## Project licensing status

The copied ROBOTIS model's license is explicitly retained. The team has not yet
selected a repository-wide license for its own code and documentation. No blanket
license for all project materials is implied by the upstream model's license.
Choose a project license deliberately if public reuse permissions are intended;
do not delete or replace third-party notices when doing so.
