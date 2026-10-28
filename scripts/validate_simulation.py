import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import contextlib
import tempfile
import main as m
import numpy as np
if '--turtlebot' in sys.argv:
    import turtlebot
    turtlebot.configure()

# Verify the camera model Jacobian against independent finite differences.
s = np.array([0.2, -0.3, 0.4])
_, h = m.camera_prediction_and_jacobian(s)
numeric = np.column_stack([(m.camera_prediction_and_jacobian(s + np.eye(3)[i] * 1e-6)[0] - m.camera_prediction_and_jacobian(s - np.eye(3)[i] * 1e-6)[0]) / 2e-6 for i in range(3)])
assert np.allclose(h, numeric, atol=1e-6)
# Verify rendered camera measurements at known validation poses.
for x, y, theta in [(1., 0., 0.), (1.4, .1, -.1), (.7, -.2, .1)]:
    m.true_x, m.true_y, m.true_theta = x, y, theta
    m.update_mujoco_pose()
    found, r, b = m.detect_aruco_measurement(m.camera_image())
    predicted, _ = m.camera_prediction_and_jacobian(np.array([x, y, theta]))
    print('camera check', found, r, b, 'expected', predicted)
    print('camera heading',m.camera_heading_measurement,'expected',theta)
    assert abs(m.wrap_angle(m.camera_heading_measurement-theta)) < .15
    assert found and abs(r-predicted[0]) < .05 and abs(b-predicted[1]) < .02
if '--camera-only' in sys.argv:
    m.renderer.close()
    print('Camera measurement, heading and Jacobian checks passed.')
    raise SystemExit(0)
m.true_x, m.true_y, m.true_theta = -2.05, -1.35, 0.
m.update_mujoco_pose()
class Viewer:
    def __init__(self):
        import mujoco
        self.cam = type('Cam', (), {'lookat': np.zeros(3)})()
        self.user_scn = mujoco.MjvScene(m.model, maxgeom=10)
        self.steps = 0
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def lock(self): return contextlib.nullcontext()
    def is_running(self):
        self.steps += 1
        if self.steps % 1000 == 0:
            print('progress', self.steps, 'actual', m.true_x, m.true_y, m.true_theta, 'estimated', m.x_hat, flush=True)
        return self.steps <= 7000 and not (m.log_rows and m.log_rows[-1][-1] == 'DOCKED')
    def sync(self): pass
    def set_texts(self, text): pass
m.mujoco.viewer.launch_passive = lambda *args: Viewer()
m.time.sleep = lambda *args: None
m.LOG_FILE = Path(tempfile.gettempdir()) / ('turtlebot_validation.csv' if '--turtlebot' in sys.argv else 'docking_validation.csv')
m.main(show_camera_window=False)
last = m.log_rows[-1]
error = np.hypot(m.true_x - m.DOCK_X, m.true_y - m.DOCK_Y)
print('RESULT', last, 'actual target error', error, 'detections', sum(r[-2] for r in m.log_rows))
assert last[-1] == 'DOCKED' and error < .15
