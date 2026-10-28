"""Fast geometry checks without a graphics context or simulator import."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from navigation import Navigator

n = Navigator(np.radians(np.arange(-180,180,5)),lambda x,y:(.1,0,1))
assert abs(n.clearance(np.array([[.6,0.]]),[0])[0]-.2)<1e-9
assert n.guard(np.array([[.43,0.]]),.16,0.,'test')[0]==0
assert n.guard(np.array([[-.43,0.]]),-.16,0.,'test')[0]==0
assert n.clearance(np.array([[.6,0.]]),[np.pi])[0]>1
# A solid front barrier requires a lateral path segment, not a straight shortcut.
points = np.column_stack([np.full(31,.8),np.linspace(-.6,.6,31)])
state = np.array([0.,0.,0.])
n.predock=np.array([2.,0.])
waypoint=n.plan(state,points)
assert waypoint is not None
assert abs(waypoint[1])>.10
direction=np.arctan2(waypoint[1],waypoint[0])
assert n.clearance(points,[direction])[0]>np.linalg.norm(waypoint)
# Crossing the old distance boundary must not cancel marker acquisition.
n=Navigator(np.radians(np.arange(-180,180,5)),lambda x,y:(.1,0,1))
clear=np.full(72,99.)
assert n.command(np.array([.95,-.19,1.]),clear,False,-1.)[-1]=='ACQUIRING DOCK MARKER'
assert n.command(np.array([.95,-.22,1.]),clear,False,-1.)[-1]=='ACQUIRING DOCK MARKER'
# Lateral error triggers a bounded reverse manoeuvre, then a fresh approach.
n.docking=True
assert n.command(np.array([1.5,.12,0.]),clear,True,0.)[0]<0
assert n.command(np.array([1.0,.04,0.]),clear,True,0.)[0]>0
print('Footprint, braking, path, acquisition latch and docking recovery checks passed.')
