import csv
import math
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
ARUCO_SIZE_M = 0.50

WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.160
MAX_WHEEL_SPEED = 9.0

MAX_SPEED = 0.26
MAX_TURN = 1.25

FRONT_LIMIT = 0.68
CLEAR_LIMIT = 0.90
SIDE_CLEAR_LIMIT = 0.55

PREDOCK_X = 1.55
PREDOCK_Y = 0.00
DOCK_X = 2.10
DOCK_Y = 0.00
DOCK_HEADING = 0.0
DOCK_DISTANCE = 0.38

# Known landmark location used by the EKF camera measurement model.
LANDMARK_X = 2.45
LANDMARK_Y = 0.00

# Encoder noise is applied to wheel travel measurements, not to ground truth motion.
ENCODER_STD_M = 0.0007
CAMERA_RANGE_STD_M = 0.05
CAMERA_BEARING_STD_RAD = math.radians(2.0)

# EKF covariance matrices.
EKF_Q = np.diag([0.0008, 0.0008, math.radians(0.8) ** 2])
EKF_R = np.diag([CAMERA_RANGE_STD_M ** 2, CAMERA_BEARING_STD_RAD ** 2])

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

    top_width = np.linalg.norm(points[1] - points[0])
    bottom_width = np.linalg.norm(points[2] - points[3])
    marker_width_px = max(1.0, float((top_width + bottom_width) / 2.0))

    bearing = math.atan2(
        centre_x - CAMERA_WIDTH / 2.0,
        focal_x
    )

    range_m = (focal_x * ARUCO_SIZE_M) / marker_width_px

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

    theta_mid = x_hat[2] + dtheta / 2.0

    x_hat[0] += ds * math.cos(theta_mid)
    x_hat[1] += ds * math.sin(theta_mid)
    x_hat[2] = wrap_angle(x_hat[2] + dtheta)

    F = np.array([
        [1.0, 0.0, -ds * math.sin(theta_mid)],
        [0.0, 1.0,  ds * math.cos(theta_mid)],
        [0.0, 0.0, 1.0]
    ])

    P = F @ P @ F.T + EKF_Q


def camera_prediction_and_jacobian(state):
    dx = LANDMARK_X - state[0]
    dy = LANDMARK_Y - state[1]
    q = max(dx * dx + dy * dy, 1e-9)
    r = math.sqrt(q)

    predicted_range = r
    predicted_bearing = wrap_angle(math.atan2(dy, dx) - state[2])

    H = np.array([
        [-dx / r, -dy / r, 0.0],
        [ dy / q, -dx / q, -1.0]
    ])

    return np.array([predicted_range, predicted_bearing]), H


def ekf_camera_update(measured_range, measured_bearing):
    global x_hat, P

    z = np.array([measured_range, measured_bearing])
    z_hat, H = camera_prediction_and_jacobian(x_hat)

    innovation = z - z_hat
    innovation[1] = wrap_angle(innovation[1])

    S = H @ P @ H.T + EKF_R
    K = P @ H.T @ np.linalg.inv(S)

    x_hat = x_hat + K @ innovation
    x_hat[2] = wrap_angle(x_hat[2])

    I = np.eye(3)
    P = (I - K @ H) @ P


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

    if final_heading is None or distance > 0.35:
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
        v = nominal_speed + float(correction[0])
        omega = float(correction[1])
    except Exception:
        # Stable fallback if the local Riccati solve becomes ill-conditioned.
        heading_error = wrap_angle(target_heading - x_hat[2])
        v = min(MAX_SPEED, 0.35 * distance)
        omega = 1.4 * heading_error

    v = float(np.clip(v, 0.0, MAX_SPEED))
    omega = float(np.clip(omega, -MAX_TURN, MAX_TURN))

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


def main():
    avoiding = False
    avoidance_direction = 0
    docked = False
    start_run = time.time()

    analyse_model()

    with mujoco.viewer.launch_passive(model, data) as viewer:
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

            if marker_found:
                noisy_range = measured_range + RNG.normal(0.0, CAMERA_RANGE_STD_M)
                noisy_bearing = measured_bearing + RNG.normal(0.0, CAMERA_BEARING_STD_RAD)
                ekf_camera_update(noisy_range, noisy_bearing)

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

            if docked:
                v_cmd = 0.0
                w_cmd = 0.0
                mode = "DOCKED"

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

            elif distance_to_predock > 0.45:
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

            if distance_to_dock <= DOCK_DISTANCE:
                docked = True
                v_cmd = 0.0
                w_cmd = 0.0
                mode = "DOCKED"

            # CONTROLLER OUTPUT -> WHEEL COMMANDS -> PLANT
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

            cv2.imshow("Robot Camera", image)
            viewer.sync()

            log_step(time.time() - start_run, mode, front, marker_found)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            elapsed = time.time() - loop_start
            if elapsed < DT:
                time.sleep(DT - elapsed)

    write_log()
    cv2.destroyAllWindows()
    renderer.close()


# SECTION 12 — START

if __name__ == "__main__":
    main()
