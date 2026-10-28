"""Run isolated, seeded TurtleBot3 trials; preserve demo logs. No GUI required.

python scripts/run_robustness.py --seeds 7 19 --max-seconds 140
"""
import argparse
import csv
import json
import hashlib
from importlib.metadata import version
import math
from pathlib import Path
import subprocess
import shutil
import sys
from datetime import datetime

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
CASES = ('baseline', 'start_pose', 'high_noise', 'push')


def trial(args):
    import numpy as np
    import turtlebot
    from scripts.headless_viewer import HeadlessViewer
    p = turtlebot.pipeline
    rng = np.random.default_rng(args.seed + 1000)
    spawn = [-2.05, -1.35, 0.]
    if args.case == 'start_pose':
        spawn = [float(rng.uniform(-2.25, -1.85)), float(rng.uniform(-1.65, -1.10)),
                 float(rng.uniform(-math.radians(15), math.radians(15)))]
    turtlebot.configure(spawn)
    p.RNG = np.random.default_rng(args.seed)
    if args.case == 'high_noise':
        p.GYRO_STD_RAD_S *= 2
        p.ENCODER_STD_M *= 2
        p.CAMERA_RANGE_STD_M *= 2
        p.CAMERA_BEARING_STD_RAD *= 2
        p.CAMERA_HEADING_STD_RAD *= 2
        p.EKF_R = np.diag([p.CAMERA_RANGE_STD_M**2, p.CAMERA_BEARING_STD_RAD**2])
    start_time = p.data.time  # Exclude initial settling.
    robot_body = p.model.body('base').id
    robot_bodies = {robot_body}
    for i in range(p.model.nbody):
        if p.model.body_parentid[i] in robot_bodies:
            robot_bodies.add(i)
    hazard_geoms = {g for g in range(p.model.ngeom)
                    if p.model.geom(g).name.startswith(('obstacle_', 'wall_', 'dock_back'))}
    touched = set()
    contact_steps = 0
    force_steps = 0

    def observe(model, data):
        nonlocal contact_steps
        contact = False
        for c in data.contact:
            if c.dist > 0:
                continue
            for obstacle, robot in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if obstacle in hazard_geoms and model.geom_bodyid[robot] in robot_bodies:
                    touched.add(model.geom(obstacle).name)
                    contact = True
        contact_steps += int(contact)

    turtlebot.physics_step_observer = observe
    original_move = p.move_true_robot

    def move(left, right):
        nonlocal force_steps
        t = p.data.time - start_time
        force = 2.0 if args.case == 'push' and 12 <= t < 12.3 else 0.
        p.data.xfrc_applied[robot_body, 1] = force  # World +y force, N.
        force_steps += int(force != 0)
        original_move(left, right)

    p.move_true_robot = move
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    p.LOG_FILE = out / f'{args.case}_seed{args.seed}.csv'

    def finished():
        if p.log_rows and len(p.log_rows) % 500 == 0:
            print(f'Progress: {len(p.log_rows)*p.DT:.0f} simulated s; '
                  f'actual=({p.true_x:.2f}, {p.true_y:.2f}, {p.true_theta:.2f}); '
                  f'estimate={p.x_hat}; mode={p.log_rows[-1][-1]}; '
                  f'command={p.LAST_COMMAND}; waypoint={p.navigator.waypoint}', flush=True)
        return bool(p.log_rows and p.log_rows[-1][-1] == 'DOCKED')

    p.main(show_camera_window=False, realtime=False,
           viewer_factory=lambda model, data: HeadlessViewer(
               model, math.ceil(args.max_seconds / p.DT), finished))
    rows = p.log_rows
    actual_error = math.hypot(p.true_x-p.DOCK_X, p.true_y-p.DOCK_Y)
    heading = abs(math.degrees(p.wrap_angle(p.true_theta-p.DOCK_HEADING)))
    docked = bool(rows and rows[-1][-1] == 'DOCKED')
    errors = [math.hypot(r[1]-r[4], r[2]-r[5]) for r in rows]
    result = dict(case=args.case, seed=args.seed, spawn=spawn,
                  docked=docked, success=docked and actual_error <= .15 and heading <= 15 and not touched,
                  completion_sim_s=float(p.data.time-start_time) if docked else None,
                  elapsed_sim_s=float(p.data.time-start_time), actual_target_error_m=actual_error,
                  actual_heading_error_deg=heading, position_rmse_m=float(np.sqrt(np.mean(np.square(errors)))),
                  collision_physics_steps=contact_steps, contacted_geoms=sorted(touched),
                  push_control_steps=force_steps, samples=len(rows), log=p.LOG_FILE.name)
    (out / f'{args.case}_seed{args.seed}.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)


def suite(args):
    out = (Path(args.output) if args.output else ROOT / 'output' / ('robustness_' + datetime.now().strftime('%Y%m%d_%H%M%S'))).resolve()
    out.mkdir(parents=True, exist_ok=False)
    metadata = dict(cases=args.cases, seeds=args.seeds, max_seconds=args.max_seconds,
                    success_definition='Controller DOCKED, actual target error <=0.15 m, actual heading error <=15 deg, zero observed hazard contact.',
                    simulation_timestep_s=.002, control_timestep_s=.02,
                    high_noise='2x encoder, camera range, bearing, heading and gyro std; matched EKF noise covariance.',
                    sensing='72-ray 360-degree planar LiDAR; wheel encoders; camera pose; unbiased gyro with 0.02 rad/s white noise (doubled in high_noise).',
                    push='World +y 2 N base force, simulation time [12,12.3) s.',
                    start_pose='Known initial pose: x U[-2.25,-1.85], y U[-1.65,-1.10], yaw U[-15,15] deg.',
                    platform=sys.platform, python=sys.version,
                    packages={name: version(name) for name in ('mujoco', 'numpy', 'scipy', 'opencv-contrib-python')},
                    source_sha256={name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                                   for name in ('main.py', 'turtlebot.py', 'navigation.py', 'uncertainty.py', 'scripts/build_turtlebot_scene.py',
                                                'scripts/run_robustness.py', 'scripts/headless_viewer.py')})
    for name in metadata['source_sha256']:
        target = out / 'source' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name,target)
    (out / 'config.json').write_text(json.dumps(metadata, indent=2))
    results = []
    for case in args.cases:
        for seed in args.seeds:
            print(f'Running {case}, seed {seed}...', flush=True)
            with (out / f'{case}_seed{seed}.txt').open('w') as transcript:
                completed = subprocess.run([sys.executable, '-u', str(Path(__file__).resolve()), '--trial',
                                            '--case', case, '--seed', str(seed), '--max-seconds', str(args.max_seconds),
                                            '--output', str(out)], cwd=ROOT, stdout=transcript, stderr=subprocess.STDOUT)
            path = out / f'{case}_seed{seed}.json'
            if completed.returncode != 0 or not path.exists():
                result = dict(case=case, seed=seed, success=False, error='Trial execution failed; see transcript.')
            else:
                result = json.loads(path.read_text())
            results.append(result)
            fields = list(dict.fromkeys(k for row in results for k in row))
            with (out / 'summary.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(results)
            lines = ['# TurtleBot3 robustness results', '',
                     'Success requires camera-confirmed controller docking, actual position error <=15 cm, '
                     'heading error <=15 degrees and no robot contact with obstacles, walls or dock.', '',
                     '| Condition | Completed trials | Successes | Success rate |', '|---|---:|---:|---:|']
            for c in args.cases:
                selected = [r for r in results if r['case'] == c]
                if selected:
                    count = sum(r['success'] for r in selected)
                    lines.append(f'| {c} | {len(selected)} | {count} | {count/len(selected):.0%} |')
            lines += ['', 'Elapsed/completion times in summary.csv are simulated seconds, excluding settling. '
                      'Per-step logs retain wall-clock time. Contact is checked every physics step. '
                      'These small samples are exploratory evidence, not a reliable population success-rate estimate. '
                      'Noise covariance is adjusted for the high-noise case; unknown-noise mismatch is not tested. '
                      'Initial pose is known; initial localisation error is not tested. '
                      'Windows requires a separate runtime check. See config.json for conditions.', '']
            (out / 'summary.md').write_text('\n'.join(lines))
            print(f"  {'PASS' if result['success'] else 'FAIL'}", flush=True)
    print(f'Results: {out}', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seeds', nargs='+', type=int, default=[7, 19])
    parser.add_argument('--cases', nargs='+', choices=CASES, default=list(CASES))
    parser.add_argument('--max-seconds', type=float, default=140.)
    parser.add_argument('--output')
    parser.add_argument('--trial', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--case', choices=CASES)
    parser.add_argument('--seed', type=int, default=7)
    args = parser.parse_args()
    if args.max_seconds <= 0 or not math.isfinite(args.max_seconds):
        parser.error('--max-seconds must be finite and positive')
    if len(set(args.seeds)) != len(args.seeds) or len(set(args.cases)) != len(args.cases):
        parser.error('Seeds and cases must be unique')
    if args.trial:
        if args.case is None or args.output is None:
            parser.error('Internal trial requires --case and --output')
        trial(args)
    else:
        suite(args)
