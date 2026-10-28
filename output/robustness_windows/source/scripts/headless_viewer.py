"""Offscreen viewer adapter: runs the same controller loop without desktop windows."""
import contextlib
from types import SimpleNamespace
import mujoco
import numpy as np


class HeadlessViewer:
    def __init__(self, model, max_steps, finished):
        self.cam = SimpleNamespace(lookat=np.zeros(3))
        self.user_scn = mujoco.MjvScene(model, maxgeom=128)
        self.max_steps = max_steps
        self.finished = finished
        self.steps = 0

    def __enter__(self):
        return self

    def __exit__(self, *_):
        return False

    def lock(self):
        return contextlib.nullcontext()

    def is_running(self):
        if self.steps >= self.max_steps or self.finished():
            return False
        self.steps += 1
        return True

    def sync(self):
        pass

    def set_texts(self, _):
        pass
