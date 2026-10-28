import csv
import math
import sys
import time
from pathlib import Path

import cv2
import mujoco
import mujoco.viewer
import numpy as np
from scipy.linalg import solve_discrete_are


# SECTION 1 — SETTINGS

DT = 0.02

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FOV_Y = math.radians(60.0)
ARUCO_SIZE_M = 0.68 * 700 / 980  # Black square, excluding the generated white margin.
CAMERA_OFFSET_M = 0.18

WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.160
MAX_WHEEL_SPEED = 9.0

MAX_SPEED = 0.26
MAX_TURN = 1.25

FRONT_LIMIT = 0.68
CLEAR_LIMIT = 0.90
SIDE_CLEAR_LIMIT = 0.55

PREDOCK_X = 0.95
PREDOCK_Y = 0.00
DOCK_X = 1.55
DOCK_Y = 0.00
DOCK_HEADING = 0.0
DOCK_DISTANCE = 0.10
FINAL_ALIGNMENT_DISTANCE = 0.08
NAVIGATION_POLICY = None
DOCK_HEADING_TOLERANCE = math.radians(12)
LAST_COMMAND = (0., 0.)
YAW_INCREMENT = None
YAW_INCREMENT_VARIANCE = 0.

# Known landmark location used by the EKF camera measurement model.
LANDMARK_X = 2.45 - 0.059
LANDMARK_Y = 0.00

# Encoder noise is applied to wheel travel measurements, not to ground truth motion.
ENCODER_STD_M = 0.0007
CAMERA_RANGE_STD_M = 0.05
CAMERA_BEARING_STD_RAD = math.radians(2.0)
CAMERA_HEADING_STD_RAD = math.radians(8.0)
camera_heading_measurement = None
USE_CAMERA_HEADING = False

# Camera covariance; prediction covariance is derived from wheel noise below.
EKF_R = np.diag([CAMERA_RANGE_STD_M ** 2, CAMERA_BEARING_STD_RAD ** 2])
PLANT_PROCESS_COV = np.zeros((3, 3))

# LQR penalties.
LQR_Q = np.diag([7.0, 7.0, 2.5])
LQR_R = np.diag([1.0, 0.55])

RNG = np.random.default_rng(7)


# SECTION 2 — LOAD MUJOCO

PROJECT_FOLDER = Path(__file__).resolve().parent
SCENE_FILE = PROJECT_FOLDER / "scene.xml"
LOG_FILE = PROJECT_FOLDER / "run_log.csv"

model = mujoco.MjModel.from_xml_path(str(SCENE_FILE))
data = mujoco.MjData(model)

robot_joint = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_JOINT,
    "robot_free"
)
robot_qpos = model.jnt_qposadr[robot_joint]


# SECTION 3 — TRUE PLANT STATE

# Ground truth exists only to move/render the simulated plant and evaluate the EKF.
# The LQR controller never reads these values directly.
true_x = -2.05
true_y = -1.35
true_theta = 0.0


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def update_mujoco_pose():
    data.qpos[robot_qpos + 0] = true_x
    data.qpos[robot_qpos + 1] = true_y
    data.qpos[robot_qpos + 2] = 0.13

    data.qpos[robot_qpos + 3:robot_qpos + 7] = [
        math.cos(true_theta / 2.0),
        0.0,
        0.0,
        math.sin(true_theta / 2.0)
    ]

    mujoco.mj_forward(model, data)


update_mujoco_pose()


# SECTION 4 — NATIVE MUJOCO RANGE SENSORS

SENSOR_NAMES = [
    "range_m90", "range_m75", "range_m60", "range_m45", "range_m30", "range_m15",
    "range_0",
    "range_p15", "range_p30", "range_p45", "range_p60", "range_p75", "range_p90"
]

sensor_addresses = []
for name in SENSOR_NAMES:
    sensor_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SENSOR,
        name
    )
    sensor_addresses.append(model.sensor_adr[sensor_id])


def read_range_sensors():
    readings = []

    for address in sensor_addresses:
        distance = float(data.sensordata[address])
        if distance < 0:
            distance = 99.0
        readings.append(distance)

    return np.array(readings)


def sensor_regions(readings):
    right = float(np.min(readings[0:5]))
    front = float(np.min(readings[5:8]))
    left = float(np.min(readings[8:13]))
    return front, left, right


# SECTION 5 — CAMERA + ARUCO MEASUREMENT

renderer = mujoco.Renderer(
    model,
    width=CAMERA_WIDTH,
    height=CAMERA_HEIGHT
)

aruco_dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)
aruco_detector = cv2.aruco.ArucoDetector(
    aruco_dictionary,
    cv2.aruco.DetectorParameters()
)

# Approximate horizontal focal length from the vertical FOV and image aspect ratio.
focal_y = (CAMERA_HEIGHT / 2.0) / math.tan(CAMERA_FOV_Y / 2.0)
focal_x = focal_y


def camera_image():
    renderer.update_scene(data, camera="robot_camera")
    rgb = renderer.render()
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def _detect_marker_once(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco_detector.detectMarkers(gray)

    if ids is None:
        return False, None

    for marker_corners, marker_id in zip(corners, ids.flatten()):
        if int(marker_id) == 0:
            return True, marker_corners.reshape(4, 2)

    return False, None


def detect_aruco_measurement(image):
    global camera_heading_measurement
    camera_heading_measurement = None
    found, points = _detect_marker_once(image)
    mirrored = False

    if not found:
        flipped = cv2.flip(image, 1)
        found, points = _detect_marker_once(flipped)
        mirrored = found

    if not found:
        return False, None, None

    if mirrored:
        points = points.copy()
        points[:, 0] = CAMERA_WIDTH - points[:, 0]

    centre_x = float(np.mean(points[:, 0]))
    half = ARUCO_SIZE_M / 2
    object_points = np.array([[-half, half, 0], [half, half, 0],
                              [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    camera_matrix = np.array([[focal_x, 0, CAMERA_WIDTH / 2],
                              [0, focal_y, CAMERA_HEIGHT / 2], [0, 0, 1]])
    success, rotation, translation = cv2.solvePnP(
        object_points, points.astype(np.float64), camera_matrix,
        np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not success or not np.all(np.isfinite(translation)) or translation[2, 0] <= 0:
        return False, None, None
    # OpenCV right is robot -y; EKF bearings are positive to the left.
    lateral, _, forward = translation.flatten()
    bearing = math.atan2(-lateral, forward)
    range_m = math.hypot(lateral, forward)
    normal = cv2.Rodrigues(rotation)[0][:, 2]
    if normal[2] > 0:
        normal = -normal
    # The station texture's horizontal mapping reverses its normal x component.
    camera_heading_measurement = -math.atan2(normal[0], -normal[2])

    cv2.polylines(
        image,
        [points.astype(np.int32)],
        True,
        (0, 255, 0),
        2
    )

    cv2.circle(
        image,
        (int(centre_x), int(np.mean(points[:, 1]))),
        5,
        (0, 255, 0),
        -1
    )

    return True, range_m, bearing


# SECTION 6 — DIFFERENTIAL DRIVE + WHEEL ENCODERS


def body_to_wheels(v, omega):
    left = (v - omega * WHEEL_BASE / 2.0) / WHEEL_RADIUS
    right = (v + omega * WHEEL_BASE / 2.0) / WHEEL_RADIUS

    left = float(np.clip(left, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))
    right = float(np.clip(right, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))

    return left, right


def wheel_to_body(left_omega, right_omega):
    v = WHEEL_RADIUS * (right_omega + left_omega) / 2.0
    omega = WHEEL_RADIUS * (right_omega - left_omega) / WHEEL_BASE
    return v, omega


def move_true_robot(left_omega, right_omega):
    global true_x, true_y, true_theta

    v, omega = wheel_to_body(left_omega, right_omega)

    true_x += v * math.cos(true_theta) * DT
    true_y += v * math.sin(true_theta) * DT
    true_theta = wrap_angle(true_theta + omega * DT)

    update_mujoco_pose()


def encoder_measurement(left_omega, right_omega):
    left_distance = WHEEL_RADIUS * left_omega * DT
    right_distance = WHEEL_RADIUS * right_omega * DT

    measured_left = left_distance + RNG.normal(0.0, ENCODER_STD_M)
    measured_right = right_distance + RNG.normal(0.0, ENCODER_STD_M)

    return measured_left, measured_right


# SECTION 7 — EKF STATE ESTIMATOR

# x_hat = [x, y, theta].
x_hat = np.array([-2.05, -1.35, 0.0], dtype=float)
P = np.diag([0.03, 0.03, math.radians(5.0) ** 2])


def ekf_predict(left_distance, right_distance):
    global x_hat, P

    ds = (right_distance + left_distance) / 2.0
    dtheta = (right_distance - left_distance) / WHEEL_BASE
    if YAW_INCREMENT is not None:
        dtheta = YAW_INCREMENT

    theta_mid = x_hat[2] + dtheta / 2.0

    x_hat[0] += ds * math.cos(theta_mid)
    x_hat[1] += ds * math.sin(theta_mid)
    x_hat[2] = wrap_angle(x_hat[2] + dtheta)

    F = np.array([
        [1.0, 0.0, -ds * math.sin(theta_mid)],
        [0.0, 1.0,  ds * math.cos(theta_mid)],
        [0.0, 0.0, 1.0]
    ])

    G = np.array([[0.5 * math.cos(theta_mid) + ds * math.sin(theta_mid) / (2 * WHEEL_BASE),
                   0.5 * math.cos(theta_mid) - ds * math.sin(theta_mid) / (2 * WHEEL_BASE)],
                  [0.5 * math.sin(theta_mid) - ds * math.cos(theta_mid) / (2 * WHEEL_BASE),
                   0.5 * math.sin(theta_mid) + ds * math.cos(theta_mid) / (2 * WHEEL_BASE)],
                  [-1 / WHEEL_BASE, 1 / WHEEL_BASE]])
    if YAW_INCREMENT is not None:
        G = np.array([[.5*math.cos(theta_mid), .5*math.cos(theta_mid)],
                      [.5*math.sin(theta_mid), .5*math.sin(theta_mid)], [0.,0.]])
    P = F @ P @ F.T + G @ (np.eye(2) * ENCODER_STD_M ** 2) @ G.T + PLANT_PROCESS_COV
    if YAW_INCREMENT is not None:
        gyro_jacobian = np.array([-.5*ds*math.sin(theta_mid), .5*ds*math.cos(theta_mid), 1.])
        P += np.outer(gyro_jacobian,gyro_jacobian)*YAW_INCREMENT_VARIANCE


def camera_prediction_and_jacobian(state):
    dx = LANDMARK_X - state[0] - CAMERA_OFFSET_M * math.cos(state[2])
    dy = LANDMARK_Y - state[1] - CAMERA_OFFSET_M * math.sin(state[2])
    q = max(dx * dx + dy * dy, 1e-9)
    r = math.sqrt(q)

    predicted_range = r
    predicted_bearing = wrap_angle(math.atan2(dy, dx) - state[2])

    H = np.array([
        [-dx / r, -dy / r, 0.0],
        [ dy / q, -dx / q, -1.0]
    ])
    H[0, 2] = CAMERA_OFFSET_M * (dx * math.sin(state[2]) - dy * math.cos(state[2])) / r
    H[1, 2] = -1 - CAMERA_OFFSET_M * (dy * math.sin(state[2]) + dx * math.cos(state[2])) / q

    return np.array([predicted_range, predicted_bearing]), H


def ekf_camera_update(measured_range, measured_bearing):
    global x_hat, P

    z = np.array([measured_range, measured_bearing])
    z_hat, H = camera_prediction_and_jacobian(x_hat)
    R = EKF_R
    if USE_CAMERA_HEADING and camera_heading_measurement is not None:
        z = np.append(z, wrap_angle(camera_heading_measurement + RNG.normal(0, CAMERA_HEADING_STD_RAD)))
        z_hat = np.append(z_hat, x_hat[2])
        H = np.vstack([H, [0, 0, 1]])
        R = np.diag([CAMERA_RANGE_STD_M**2, CAMERA_BEARING_STD_RAD**2, CAMERA_HEADING_STD_RAD**2])

    innovation = z - z_hat
    innovation[1] = wrap_angle(innovation[1])
    if len(innovation) == 3:
        innovation[2] = wrap_angle(innovation[2])

    S = H @ P @ H.T + R
    if innovation @ np.linalg.solve(S, innovation) > (11.34 if len(innovation) == 3 else 9.21):
        return False
    K = P @ H.T @ np.linalg.inv(S)

    x_hat = x_hat + K @ innovation
    x_hat[2] = wrap_angle(x_hat[2])

    I = np.eye(3)
    residual = I - K @ H
    P = residual @ P @ residual.T + K @ R @ K.T
    return True


# SECTION 8 — STATE-SPACE MODEL + LQR


def linearised_model(theta, nominal_speed):
    A = np.array([
        [1.0, 0.0, -nominal_speed * math.sin(theta) * DT],
        [0.0, 1.0,  nominal_speed * math.cos(theta) * DT],
        [0.0, 0.0, 1.0]
    ])

    B = np.array([
        [math.cos(theta) * DT, 0.0],
        [math.sin(theta) * DT, 0.0],
        [0.0, DT]
    ])

    return A, B


def lqr_gain(A, B):
    P_lqr = solve_discrete_are(A, B, LQR_Q, LQR_R)
    return np.linalg.inv(B.T @ P_lqr @ B + LQR_R) @ (B.T @ P_lqr @ A)


def lqr_to_target(target_x, target_y, final_heading=None):
    # IMPORTANT: controller uses x_hat only, never true_x/true_y/true_theta.
    dx = target_x - x_hat[0]
    dy = target_y - x_hat[1]
    distance = math.hypot(dx, dy)

    if final_heading is None or distance > FINAL_ALIGNMENT_DISTANCE:
        target_heading = math.atan2(dy, dx)
    else:
        target_heading = final_heading

    reference = np.array([target_x, target_y, target_heading])
    error = x_hat - reference
    error[2] = wrap_angle(error[2])

    nominal_speed = min(MAX_SPEED, max(0.08, 0.45 * distance))
    A, B = linearised_model(x_hat[2], nominal_speed)

    try:
        K = lqr_gain(A, B)
        correction = -K @ error
        v = float(correction[0])
        omega = float(correction[1])
    except Exception:
        # Stable fallback if the local Riccati solve becomes ill-conditioned.
        heading_error = wrap_angle(target_heading - x_hat[2])
        v = min(MAX_SPEED, 0.35 * distance)
        omega = 1.4 * heading_error

    v = float(np.clip(v, 0.0, MAX_SPEED))
    omega = float(np.clip(omega, -MAX_TURN, MAX_TURN))
    # Turn towards the target before translating; the forward-only plant
    # cannot follow a local LQR solution when the goal is behind it.
    heading_error = wrap_angle(target_heading - x_hat[2])
    if abs(heading_error) > 0.45:
        v = 0.0
        omega = float(np.clip(1.8 * heading_error, -MAX_TURN, MAX_TURN))

    return v, omega, distance


def analyse_model():
    A, B = linearised_model(0.0, 0.20)
    controllability = np.hstack([B, A @ B, A @ A @ B])
    controllability_rank = np.linalg.matrix_rank(controllability)

    _, H = camera_prediction_and_jacobian(np.array([-1.0, -0.6, 0.0]))
    observability = np.vstack([H, H @ A, H @ A @ A])
    observability_rank = np.linalg.matrix_rank(observability)

    print(f"Controllability rank: {controllability_rank}/3")
    print(f"Observability rank:    {observability_rank}/3")


# SECTION 9 — LIDAR OBSTACLE AVOIDANCE


def obstacle_avoidance(front, left, right, direction):
    if direction == 0:
        direction = 1 if left > right else -1

    if front < FRONT_LIMIT:
        return 0.04, direction * MAX_TURN, direction

    # Continue curving around the obstacle until the front and sides are clear.
    return 0.12, direction * 0.42, direction


# SECTION 10 — LOGGING

log_rows = []


def log_step(t, mode, front, marker_found):
    log_rows.append([
        t,
        true_x, true_y, true_theta,
        x_hat[0], x_hat[1], x_hat[2],
        front,
        int(marker_found),
        mode
    ])


def write_log():
    with LOG_FILE.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([
            "time_s",
            "true_x", "true_y", "true_theta",
            "est_x", "est_y", "est_theta",
            "front_range_m",
            "aruco_detected",
            "mode"
        ])
        writer.writerows(log_rows)

    print(f"Saved run log: {LOG_FILE}")


# SECTION 11 — MAIN LOOP


def main(show_camera_window=None, viewer_factory=None, realtime=True):
    global LAST_COMMAND
    # mjpython reserves the macOS main thread for Cocoa. OpenCV HighGUI
    # windows cannot safely run on its Python worker thread.
    if show_camera_window is None:
        show_camera_window = sys.platform != "darwin"
    if sys.platform == "darwin" and not show_camera_window:
        print("macOS: camera processing is active; separate OpenCV window disabled. Close the MuJoCo window to quit.")
    avoiding = False
    avoidance_direction = 0
    docked = False
    approaching_dock = False
    docking_frames = 0
    last_status = None
    start_run = time.time()

    analyse_model()

    launch_viewer = mujoco.viewer.launch_passive if viewer_factory is None else viewer_factory
    with launch_viewer(model, data) as viewer:
        with viewer.lock():
            label = viewer.user_scn.geoms[0]
            mujoco.mjv_initGeom(label, mujoco.mjtGeom.mjGEOM_LABEL,
                               np.zeros(3), np.array([2.45, 0, 1.3]),
                               np.eye(3).flatten(), np.array([0.2, 1, 0.2, 1]))
            label.label = "CHARGING STATION"
            viewer.user_scn.ngeom = 1
        viewer.cam.distance = 6.2
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -50
        viewer.cam.lookat[:] = np.array([0.0, 0.0, 0.0])

        previous_left_cmd = 0.0
        previous_right_cmd = 0.0

        while viewer.is_running():
            loop_start = time.time()

            # ENCODER -> EKF PREDICTION
            measured_left, measured_right = encoder_measurement(
                previous_left_cmd,
                previous_right_cmd
            )
            ekf_predict(measured_left, measured_right)

            # CAMERA -> EKF UPDATE
            image = camera_image()
            marker_found, measured_range, measured_bearing = detect_aruco_measurement(image)

            camera_accepted = False
            if marker_found:
                noisy_range = measured_range + RNG.normal(0.0, CAMERA_RANGE_STD_M)
                noisy_bearing = measured_bearing + RNG.normal(0.0, CAMERA_BEARING_STD_RAD)
                camera_accepted = ekf_camera_update(noisy_range, noisy_bearing)

            # NATIVE RANGE SENSORS
            readings = read_range_sensors()
            front, left, right = sensor_regions(readings)
            obstacle_detected = front < FRONT_LIMIT

            # All navigation decisions use the EKF estimate.
            distance_to_predock = math.hypot(
                PREDOCK_X - x_hat[0],
                PREDOCK_Y - x_hat[1]
            )
            distance_to_dock = math.hypot(
                DOCK_X - x_hat[0],
                DOCK_Y - x_hat[1]
            )
            if distance_to_predock < 0.30:
                approaching_dock = True

            if docked:
                v_cmd = 0.0
                w_cmd = 0.0
                mode = "DOCKED"

            elif NAVIGATION_POLICY is not None:
                v_cmd, w_cmd, mode = NAVIGATION_POLICY(
                    readings, camera_accepted, measured_range, measured_bearing)

            elif obstacle_detected:
                avoiding = True
                v_cmd, w_cmd, avoidance_direction = obstacle_avoidance(
                    front, left, right, avoidance_direction
                )
                mode = "LIDAR AVOIDANCE"

            elif avoiding:
                if (
                    front > CLEAR_LIMIT
                    and left > SIDE_CLEAR_LIMIT
                    and right > SIDE_CLEAR_LIMIT
                ):
                    avoiding = False
                    avoidance_direction = 0
                    v_cmd, w_cmd, _ = lqr_to_target(PREDOCK_X, PREDOCK_Y)
                    mode = "LQR RESUME"
                else:
                    v_cmd, w_cmd, avoidance_direction = obstacle_avoidance(
                        front, left, right, avoidance_direction
                    )
                    mode = "LIDAR AVOIDANCE"

            elif not approaching_dock:
                v_cmd, w_cmd, _ = lqr_to_target(PREDOCK_X, PREDOCK_Y)
                mode = "LQR NAVIGATION"

            else:
                # Near the charging station, camera measurements correct the EKF
                # and LQR tracks the final docking pose from that estimated state.
                v_cmd, w_cmd, _ = lqr_to_target(
                    DOCK_X,
                    DOCK_Y,
                    DOCK_HEADING
                )

                v_cmd = min(v_cmd, 0.10)
                mode = "VISUAL EKF + LQR DOCKING" if marker_found else "LQR DOCK APPROACH"

            # Odometry alone can stop at a fictitious target after wheel slip.
            # Search for the landmark rather than settling without visual lock.
            if NAVIGATION_POLICY is None and approaching_dock and not docked and not marker_found and distance_to_dock < 0.35:
                predicted_camera, _ = camera_prediction_and_jacobian(x_hat)
                search_bearing = predicted_camera[1]
                v_cmd = 0.0
                w_cmd = float(np.clip(1.2 * search_bearing, -.6, .6)) if abs(search_bearing) > .1 else .15
                mode = "SEARCHING FOR DOCK MARKER"

            visual_docking = (
                camera_accepted and distance_to_dock <= DOCK_DISTANCE
                and abs(wrap_angle(x_hat[2] - DOCK_HEADING)) < DOCK_HEADING_TOLERANCE
                and abs(measured_bearing) < math.radians(8)
                and abs(measured_range - (LANDMARK_X - DOCK_X - CAMERA_OFFSET_M)) < 0.12
            )
            docking_frames = docking_frames + 1 if visual_docking else 0
            if docked or docking_frames >= 15:
                docked = True
                v_cmd = 0.0
                w_cmd = 0.0
                mode = "DOCKED"

            # CONTROLLER OUTPUT -> WHEEL COMMANDS -> PLANT
            LAST_COMMAND = (v_cmd, w_cmd)
            left_cmd, right_cmd = body_to_wheels(v_cmd, w_cmd)
            move_true_robot(left_cmd, right_cmd)

            previous_left_cmd = left_cmd
            previous_right_cmd = right_cmd

            # DISPLAY
            estimate_error = math.hypot(true_x - x_hat[0], true_y - x_hat[1])

            cv2.putText(image, mode, (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2)
            cv2.putText(image, f"Obstacle: {obstacle_detected}", (15, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 255) if obstacle_detected else (0, 255, 0), 1)
            cv2.putText(image, f"Front range: {front:.2f} m", (15, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
            cv2.putText(image, f"ArUco: {marker_found}", (15, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0) if marker_found else (0, 0, 255), 1)
            cv2.putText(image, f"EKF xhat: {x_hat[0]:.2f}, {x_hat[1]:.2f}, {math.degrees(x_hat[2]):.1f} deg", (15, 126), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(image, f"Est error: {estimate_error:.3f} m", (15, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(image, f"Wheel cmd L/R: {left_cmd:.2f}, {right_cmd:.2f} rad/s", (15, 174), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

            if show_camera_window:
                cv2.imshow("Robot Camera", image)
            status = f"{mode} | ArUco: {marker_found} | camera update: {camera_accepted}"
            if mode != last_status:
                print(status)
                last_status = mode
            if hasattr(viewer, "set_texts"):
                viewer.set_texts((None, None, "Mode\nArUco / accepted\nEstimated dock distance\nEvaluation position error",
                                  f"{mode}\n{marker_found} / {camera_accepted}\n{distance_to_dock:.2f} m\n{estimate_error:.2f} m"))
            viewer.sync()

            log_step(time.time() - start_run, mode, front, marker_found)

            if show_camera_window and cv2.waitKey(1) & 0xFF == ord("q"):
                break

            elapsed = time.time() - loop_start
            if realtime and elapsed < DT:
                time.sleep(DT - elapsed)

    write_log()
    if show_camera_window:
        cv2.destroyAllWindows()
    renderer.close()


# SECTION 12 — START

if __name__ == "__main__":
    main()
