"""Offscreen regression of the interactive push demo and uncertainty graphics."""
import sys,json,math,csv
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import mujoco,cv2
import demo,turtlebot
from uncertainty import draw_viewer
from scripts.headless_viewer import HeadlessViewer

root=Path(__file__).resolve().parents[1]
out=root/'output/uncertainty_demo_validation'
out.mkdir(parents=True,exist_ok=True)
p=demo.configure_demo(7)
p.LOG_FILE=out/'demo.csv'
start=p.data.time
contacts=set();push_steps=0
hazards={i for i in range(p.model.ngeom) if p.model.geom(i).name.startswith(('obstacle_','wall_','dock_back'))}

def observe(model,data):
    global push_steps
    push_steps+=int(data.xfrc_applied[model.body('base').id,1]!=0)
    for c in data.contact:
        if c.dist<=0:
            for a,b in [(c.geom1,c.geom2),(c.geom2,c.geom1)]:
                if a in hazards and model.geom_bodyid[b]!=0:contacts.add(model.geom(a).name)

turtlebot.physics_step_observer=observe

class Viewer(HeadlessViewer):
    def __exit__(self,*args):
        camera=mujoco.MjvCamera()
        camera.lookat[:]=[1.5,0,.25];camera.distance=2.2
        camera.azimuth=135;camera.elevation=-35
        opt=mujoco.MjvOption();opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER]=False
        p.renderer.update_scene(p.data,camera=camera,scene_option=opt)
        draw_viewer(p.renderer.scene,p.x_hat,p.P,np.array([p.true_x,p.true_y,p.true_theta]),
                    first_geom=p.renderer.scene.ngeom)
        cv2.imwrite(str(out/'viewer_preview.png'),cv2.cvtColor(p.renderer.render(),cv2.COLOR_RGB2BGR))
        return False

p.main(show_camera_window=False,realtime=False,viewer_factory=lambda m,d:Viewer(
    m,7000,lambda:bool(p.log_rows and p.log_rows[-1][-1]=='DOCKED')))
assert p.log_rows[-1][-1]=='DOCKED'
error=math.hypot(p.true_x-p.DOCK_X,p.true_y-p.DOCK_Y)
heading=abs(math.degrees(p.wrap_angle(p.true_theta)))
assert error<=.15 and heading<=15 and not contacts and push_steps==150
for row in csv.DictReader(p.LOG_FILE.open()):
    c=np.array([[float(row['P_xx']),float(row['P_xy']),float(row['P_xtheta'])],
                [float(row['P_xy']),float(row['P_yy']),float(row['P_ytheta'])],
                [float(row['P_xtheta']),float(row['P_ytheta']),float(row['P_thetatheta'])]])
    assert np.isfinite(c).all() and np.linalg.eigvalsh(c).min()>-1e-10
result=dict(success=True,seed=7,sim_seconds=float(p.data.time-start),actual_error_m=error,
            actual_heading_error_deg=heading,contacted_geoms=sorted(contacts),push_physics_steps=push_steps)
(out/'result.json').write_text(json.dumps(result,indent=2))
print(result)
