"""Generate report figures from a saved run without launching MuJoCo.

Usage: python scripts/plot_results.py [path/to/run_log.csv]
"""
import ast
import argparse
import csv
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from uncertainty import ellipse_points

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('log',nargs='?',type=Path,default=ROOT/'run_log.csv')
parser.add_argument('--output',type=Path)
args = parser.parse_args()
source = args.log.resolve()
if args.output is not None:
    output = args.output.resolve()
elif source.parent.name.startswith('robustness_'):
    output = source.parent/'plots'/source.stem
else:
    output = ROOT / 'output' / ('results_turtlebot' if 'turtlebot' in source.stem else 'results')
output.mkdir(parents=True, exist_ok=True)
with source.open(newline='') as stream:
    rows = list(csv.DictReader(stream))
if not rows:
    raise SystemExit('The run log has no samples.')

def column(name):
    return np.array([float(row[name]) for row in rows])

t = column('time_s')
position = np.column_stack([column('true_x'), column('true_y')])
estimate = np.column_stack([column('est_x'), column('est_y')])
error = np.linalg.norm(estimate - position, axis=1)
heading_error = np.degrees((column('est_theta') - column('true_theta') + np.pi) % (2*np.pi) - np.pi)
detected = column('aruco_detected').astype(bool)
docked_indices = [i for i, row in enumerate(rows) if row['mode'] == 'DOCKED']
end = docked_indices[0] if docked_indices else len(rows) - 1
active = slice(0, end + 1)
has_covariance = all(k in rows[0] for k in ('P_xx','P_xy','P_yy','P_thetatheta'))
# Read the target constants without importing main.py (which creates a renderer).
settings = {}
for node in ast.parse((ROOT / 'main.py').read_text()).body:
    if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant):
        for target in node.targets:
            if isinstance(target, ast.Name):
                settings[target.id] = node.value.value
target = np.array([settings['DOCK_X'], settings['DOCK_Y']])
plt.rcParams.update({'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False})

fig, ax = plt.subplots(figsize=(8, 7), constrained_layout=True)
scene = ET.parse(ROOT / 'scene.xml')
for geom in scene.findall('.//worldbody/geom'):
    if geom.get('name', '').startswith('obstacle_'):
        x, y, _ = map(float, geom.get('pos').split())
        sx, sy, _ = map(float, geom.get('size').split())
        colour = list(map(float, geom.get('rgba').split()))
        ax.add_patch(Rectangle((x-sx, y-sy), 2*sx, 2*sy, color=colour, alpha=.55))
ax.plot(*position[active].T, label='Actual trajectory', color='#2166ac', linewidth=2)
ax.plot(*estimate[active].T, label='EKF estimate', color='#d6604d', linestyle='--', linewidth=1.5)
if has_covariance:
    for i in np.linspace(0,end,min(14,end+1),dtype=int):
        cov=np.array([[float(rows[i]['P_xx']),float(rows[i]['P_xy'])],
                      [float(rows[i]['P_xy']),float(rows[i]['P_yy'])]])
        ellipse=ellipse_points(estimate[i],cov)
        ax.plot(*ellipse.T,color='#c88a10',alpha=.65,linewidth=.8,
                label='Model joint 95% position ellipse' if i==0 else None)
ax.scatter(*position[0], marker='o', color='black', label='Start', zorder=5)
ax.scatter(*target, marker='*', s=180, color='#258642', label='Docking target', zorder=5)
ax.scatter(*position[end], marker='x', s=80, color='black', label='Position at first DOCKED' if docked_indices else 'Final position', zorder=5)
dock = scene.find(".//body[@name='dock']")
dx, dy, _ = map(float, dock.get('pos').split())
ax.plot([dx, dx], [dy-.52, dy+.52], color='#333333', linewidth=6, label='Charging station')
ax.set(xlabel='World x (m)', ylabel='World y (m)', title='Navigation trajectory through first docking' if docked_indices else 'Navigation trajectory (no DOCKED state)', xlim=(-3,3), ylim=(-3,3), aspect='equal')
ax.grid(alpha=.2)
ax.legend(loc='lower right', fontsize=9)
fig.savefig(output / 'trajectory.png', dpi=200)
plt.close(fig)

if has_covariance:
    fig,axes=plt.subplots(3,1,figsize=(10,9),sharex=True,constrained_layout=True)
    estimate_yaw=np.unwrap(column('est_theta'))
    yaw_residual=(column('true_theta')-column('est_theta')+np.pi)%(2*np.pi)-np.pi
    actual_yaw=estimate_yaw+yaw_residual
    for ax,est,actual,variance,label in zip(axes,
            [estimate[:,0],estimate[:,1],np.degrees(estimate_yaw)],
            [position[:,0],position[:,1],np.degrees(actual_yaw)],
            [column('P_xx'),column('P_yy'),column('P_thetatheta')],
            ['World x (m)','World y (m)','Yaw (deg)']):
        bound=2*np.sqrt(np.maximum(variance,0))
        if label=='Yaw (deg)':bound=np.degrees(bound)
        ax.fill_between(t,est-bound,est+bound,color='#e0a42a',alpha=.25,label='Model marginal +/-2 sigma')
        ax.plot(t,est,color='#b46b09',linewidth=1.2,label='EKF estimate')
        ax.plot(t,actual,color='#167fa0',linewidth=1.1,linestyle='--',label='True state (evaluation)')
        if docked_indices:ax.axvline(t[end],color='black',linestyle=':',linewidth=1)
        ax.set_ylabel(label);ax.grid(alpha=.2)
    axes[0].set_title('Estimated state, covariance uncertainty and true state')
    axes[0].legend(fontsize=9)
    axes[-1].set_xlabel('Elapsed wall-clock time (s)')
    fig.savefig(output/'state_uncertainty.png',dpi=200);plt.close(fig)

fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
axes[0].plot(t, error, color='#2166ac')
axes[0].set(ylabel='Position error (m)', title='EKF error and camera availability')
axes[1].plot(t, heading_error, color='#d6604d')
axes[1].set(ylabel='Heading error (deg)')
axes[2].step(t, detected.astype(int), where='post', color='#258642')
axes[2].set(ylabel='ArUco detected', xlabel='Elapsed wall-clock time (s)', yticks=[0,1], ylim=(-.1,1.1))
for ax in axes:
    ax.grid(alpha=.2)
    if docked_indices:
        ax.axvline(t[end], color='black', linestyle='--', label='First DOCKED')
if docked_indices:
    axes[0].legend()
fig.savefig(output / 'estimation_error.png', dpi=200)
plt.close(fig)

metrics = {
    'result': 'DOCKED' if docked_indices else 'No DOCKED state recorded',
    'wall_time_to_first_docked_s': float(t[end]) if docked_indices else None,
    'actual_target_error_m': float(np.linalg.norm(position[end]-target)),
    'estimated_target_error_m': float(np.linalg.norm(estimate[end]-target)),
    'actual_heading_error_deg': float(np.degrees((column('true_theta')[end]-settings['DOCK_HEADING']+np.pi) % (2*np.pi)-np.pi)),
    'position_rmse_through_docking_m': float(np.sqrt(np.mean(error[active]**2))),
    'maximum_position_error_through_docking_m': float(np.max(error[active])),
    'heading_rmse_through_docking_deg': float(np.sqrt(np.mean(heading_error[active]**2))),
    'aruco_detection_fraction_through_docking': float(np.mean(detected[active])),
    'actual_path_length_through_docking_m': float(np.linalg.norm(np.diff(position[active], axis=0), axis=1).sum()),
}
with (output / 'metrics.csv').open('w', newline='') as stream:
    writer = csv.writer(stream)
    writer.writerow(['metric', 'value'])
    writer.writerows(metrics.items())
summary = ['# Simulation run results', '', f'Source: `{source.name}`', '',
           '| Metric | Value |', '|---|---|']
for name, value in metrics.items():
    formatted = f'{value:.4f}' if isinstance(value, float) else str(value)
    summary.append(f'| {name} | {formatted} |')
summary += ['', ('Metrics use samples through the first DOCKED state, excluding subsequent idle time. ' if docked_indices else 'Metrics use the whole saved run; no DOCKED state was recorded. ')+
            'The error plot also shows the idle period. Times are logged wall-clock times, not simulated physics time. '
            'Actual target error and EKF position error are distinct quantities. Ground truth is used here only for evaluation.', '',
            'This is one seeded run, not a success-rate or robustness study. It does not establish collision-free motion. '+
            ('True pose, estimate and covariance are time-aligned before the current movement.' if has_covariance else
             'The log compares the estimate before current movement with the plant after it; errors include one control-step offset.'), '']
if has_covariance:
    summary += ['state_uncertainty.png shows marginal +/-2 sigma (approximately 95.45% for a calibrated Gaussian). '
                'Trajectory ellipses use the joint 2D 95% chi-square factor, not 2 sigma. '
                'These are model uncertainty estimates, not guaranteed error bounds or proof of filter consistency.','']
(output / 'summary.md').write_text('\n'.join(summary))
print('\n'.join(f'{key}: {value}' for key, value in metrics.items()))
print(f'Figures and summary saved to {output}')
