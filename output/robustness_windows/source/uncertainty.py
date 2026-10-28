"""Covariance visualisation only; these helpers never supply control inputs."""
import numpy as np

CHI2_95_2D = 5.991464547


def ellipse_points(mean, covariance, count=48):
    covariance = np.asarray(covariance, dtype=float)
    if covariance.shape != (2, 2) or not np.isfinite(covariance).all():
        raise ValueError('Expected a finite 2x2 position covariance')
    values, vectors = np.linalg.eigh((covariance+covariance.T)/2)
    if values.min() < -1e-10:
        raise ValueError('Position covariance is not positive semidefinite')
    angles = np.linspace(0, 2*np.pi, count+1)
    circle = np.vstack([np.cos(angles), np.sin(angles)])
    return np.asarray(mean)[:2] + (vectors @ (np.sqrt(CHI2_95_2D*np.maximum(values,0))[:,None]*circle)).T


def draw_viewer(scene, estimate, covariance, actual, first_geom=1):
    """Add evaluation graphics after the charging-station label (geometry 0)."""
    import mujoco
    points = ellipse_points(estimate, covariance[:2,:2])
    if scene.maxgeom < first_geom+len(points)-1+4:
        return
    index = first_geom
    for a,b in zip(points, points[1:]):
        geom = scene.geoms[index]
        mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),
                           np.eye(3).flatten(),np.array([1.,.65,.1,1.]))
        mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_LINE,3.,
                            np.r_[a,.30],np.r_[b,.30])
        index += 1
    for pose, colour, label in [(estimate,[1.,.65,.1,1.],'EKF + 95% position ellipse'),
                                (actual,[.1,.9,1.,1.],'True pose (evaluation)')]:
        geom = scene.geoms[index]
        mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_SPHERE,np.array([.025,0,0]),
                           np.array([pose[0],pose[1],.32]),np.eye(3).flatten(),np.array(colour))
        index += 1
        geom = scene.geoms[index]
        mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_LABEL,np.zeros(3),
                           np.array([pose[0],pose[1],.48 if label.startswith('EKF') else .39]),
                           np.eye(3).flatten(),np.array(colour))
        geom.label = label
        index += 1
    scene.ngeom = index
