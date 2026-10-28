# Third-party TurtleBot3 model

Source: https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie/tree/main/robotis_tb3

Downloaded 2026-10-28 from the repository's main-branch archive, commit
`d8344c0dbe7a00208d0301111523dde65efc174a` (archive comment). The original
`turtlebot3_waffle_pi.xml`, four referenced STL meshes and `LICENSE` are retained
unmodified here. Copyright and licensing belong to their respective authors;
see the accompanying Apache-2.0 license.

`scripts/build_turtlebot_scene.py` creates the adapted `scene_turtlebot.xml`:
project room/station, wheel velocity-servo gain/torque limit, reduced wheel
joint armature/friction, smooth cylinder tyre collision proxies, 2 ms physics timestep,
elevated simulated camera mast, a 72-ray 360-degree planar scan and a gyroscope. Wheel
geometry, mass/inertia, wheel joints and chassis collision meshes come from ROBOTIS.
The sensor additions are project simulation adaptations, not a claim to reproduce
the stock camera mounting or the full physical LDS scan characteristics.

The UTS `turtlebot_rosbridge` README describes controlling a physical robot over
rosbridge. It is not needed by this simulation-only integration.
