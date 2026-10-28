r"""TurtleBot3 wheel-contact physics adapter for the shared sensing/EKF/LQR pipeline.

macOS: ./.venv/bin/python .venv/bin/mjpython turtlebot.py
Windows: .\.venv\Scripts\python.exe turtlebot.py
"""
import math
import numpy as np
import mujoco
import main as pipeline
from scripts.build_turtlebot_scene import build
from navigation import Navigator


def configure(start_pose=(-2.05, -1.35, 0.)):
    """Select the dynamic TurtleBot plant, retaining the original demo unchanged."""
    p = pipeline
    p.renderer.close()
    p.SCENE_FILE = build()
    p.LOG_FILE = p.PROJECT_FOLDER / 'run_log_turtlebot.csv'
    p.model = mujoco.MjModel.from_xml_path(str(p.SCENE_FILE))
    p.data = mujoco.MjData(p.model)
    p.WHEEL_BASE = 0.288  # Wheel joint y positions are +/-0.144 m.
    p.MAX_WHEEL_SPEED = 7.88
    p.MAX_SPEED = 0.20
    p.USE_CAMERA_HEADING = True
    p.FINAL_ALIGNMENT_DISTANCE = 0.50
    p.DOCK_DISTANCE = .08
    p.DOCK_HEADING_TOLERANCE = math.radians(8)
    # Contact/slip uncertainty is additional to encoder measurement noise.
    p.PLANT_PROCESS_COV = np.diag([1e-5, 1e-5, math.radians(.05)**2])
    p.GYRO_STD_RAD_S = .02
    gyro_address = p.model.sensor_adr[p.model.sensor('imu_gyro').id]+2
    gyro_delta = 0.
    p.robot_joint = p.model.joint('base_joint').id
    p.robot_qpos = p.model.jnt_qposadr[p.robot_joint]
    wheel_positions = [p.model.jnt_qposadr[p.model.joint(name).id]
                       for name in ('wheel_left', 'wheel_right')]
    actuators = [p.model.actuator(name).id for name in ('wheel_left', 'wheel_right')]
    previous_wheels = np.zeros(2)

    def read_truth():
        # Evaluation only: neither encoder nor controller reads this pose.
        p.true_x, p.true_y = map(float, p.data.qpos[p.robot_qpos:p.robot_qpos+2])
        w, x, y, z = p.data.qpos[p.robot_qpos+3:p.robot_qpos+7]
        p.true_theta = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

    def reset_pose():
        nonlocal gyro_delta
        # Only initialization and known-pose camera validation use this function.
        mujoco.mj_resetData(p.model, p.data)
        p.data.qpos[p.robot_qpos:p.robot_qpos+3] = [p.true_x, p.true_y, .002]
        p.data.qpos[p.robot_qpos+3:p.robot_qpos+7] = [math.cos(p.true_theta/2), 0, 0, math.sin(p.true_theta/2)]
        for _ in range(250):
            mujoco.mj_step(p.model, p.data)
        mujoco.mj_forward(p.model, p.data)
        previous_wheels[:] = p.data.qpos[wheel_positions]
        gyro_delta = 0.
        read_truth()

    def move(left, right):
        nonlocal gyro_delta
        p.data.ctrl[actuators] = [left, right]
        angular_rates = []
        for _ in range(round(p.DT / p.model.opt.timestep)):
            mujoco.mj_step(p.model, p.data)
            angular_rates.append(float(p.data.sensordata[gyro_address]))
            if physics_step_observer is not None:
                physics_step_observer(p.model, p.data)
        mujoco.mj_forward(p.model, p.data)
        gyro_delta = float(np.mean(angular_rates))*p.DT
        read_truth()

    def encoders(_left_command, _right_command):
        # Joint rotation measurements, not commanded speed or base pose.
        angles = p.data.qpos[wheel_positions].copy()
        distance = (angles-previous_wheels) * p.WHEEL_RADIUS
        previous_wheels[:] = angles
        p.YAW_INCREMENT = gyro_delta + p.RNG.normal(0,p.GYRO_STD_RAD_S)*p.DT
        p.YAW_INCREMENT_VARIANCE = (p.GYRO_STD_RAD_S*p.DT)**2
        return tuple(distance + p.RNG.normal(0, p.ENCODER_STD_M, 2))

    p.update_mujoco_pose = reset_pose
    p.move_true_robot = move
    p.encoder_measurement = encoders
    p.SENSOR_NAMES = [f'range_{i}' for i in range(72)]
    angles = np.radians(np.arange(-180, 180, 5))
    p.sensor_addresses = [p.model.sensor_adr[p.model.sensor(name).id] for name in p.SENSOR_NAMES]
    p.sensor_regions = lambda ranges: (float(np.min(ranges[np.abs(angles) < np.pi/6])),
                                       float(np.min(ranges[(angles >= np.pi/6) & (angles <= np.pi/2)])),
                                       float(np.min(ranges[(angles <= -np.pi/6) & (angles >= -np.pi/2)])))
    navigator = Navigator(angles, p.lqr_to_target)
    p.navigator = navigator

    def navigate(ranges, accepted, measured_range, measured_bearing):
        predicted, _ = p.camera_prediction_and_jacobian(p.x_hat)
        return navigator.command(p.x_hat.copy(), ranges, accepted, float(predicted[1]))

    p.NAVIGATION_POLICY = navigate
    p.true_x, p.true_y, p.true_theta = start_pose
    reset_pose()
    p.x_hat[:] = start_pose  # Known commanded spawn; not a runtime pose measurement.
    p.P = np.diag([.03, .03, math.radians(5)**2])
    p.renderer = mujoco.Renderer(p.model, width=p.CAMERA_WIDTH, height=p.CAMERA_HEIGHT)
    print('TurtleBot3 Waffle Pi: dynamic wheel-contact plant; encoders read wheel joints.')


# Optional evaluation observer; never supplies state to the controller.
physics_step_observer = None


if __name__ == '__main__':
    configure()
    pipeline.main()
