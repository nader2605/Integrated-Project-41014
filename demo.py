"""Interactive disturbed demonstration with live EKF uncertainty.

macOS: ./.venv/bin/python .venv/bin/mjpython demo.py
Windows: .venv/Scripts/python.exe demo.py
"""
import argparse
import numpy as np
import turtlebot


def configure_demo(seed=7):
    turtlebot.configure()
    p=turtlebot.pipeline
    p.RNG=np.random.default_rng(seed)
    p.LOG_FILE=p.PROJECT_FOLDER/'run_log_demo.csv'
    start=p.data.time
    base=p.model.body('base').id
    original_move=p.move_true_robot
    applied=False

    def move(left,right):
        nonlocal applied
        elapsed=p.data.time-start
        force=2. if 12 <= elapsed < 12.3 else 0.
        p.data.xfrc_applied[base,1]=force
        if force:
            p.RUN_LABEL='PUSH ACTIVE: 2 N world +y'
            if not applied:
                print('Demo disturbance: 2 N world +y for 0.3 simulated seconds',flush=True)
            applied=True
        elif applied:
            p.RUN_LABEL='Push done: 2 N, 0.3 s at t=12 s'
        else:
            p.RUN_LABEL='Push demo: 2 N scheduled at t=12 s'
        original_move(left,right)

    p.RUN_LABEL='Push demo: 2 N scheduled at t=12 s'
    p.move_true_robot=move
    return p


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed',type=int,default=7)
    args=parser.parse_args()
    configure_demo(args.seed).main()
