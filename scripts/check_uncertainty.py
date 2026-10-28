"""Check covariance-ellipse geometry without a graphics context."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from uncertainty import ellipse_points,CHI2_95_2D

theta=.6
r=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
c=r@np.diag([.04,.01])@r.T
centre=np.array([1.2,-.3])
points=ellipse_points(centre,c)-centre
distance=np.einsum('ij,jk,ik->i',points,np.linalg.inv(c),points)
assert np.allclose(distance,CHI2_95_2D)
assert np.isfinite(ellipse_points(centre,np.diag([0.,.01]))).all()
for invalid in [np.diag([-.01,.02]),np.array([[np.nan,0],[0,1]])]:
    try:ellipse_points(centre,invalid)
    except ValueError:pass
    else:raise AssertionError('Invalid covariance was accepted')
print('Rotated 95% ellipse, singular covariance and invalid-input checks passed.')
