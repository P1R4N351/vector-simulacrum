"""Project W12 cube envelopes through the simulated head at matched SDK image stamps."""
import csv
import itertools
import json
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

here = Path(__file__).resolve().parent
web = here.parents[2]/"web"
m = mujoco.MjModel.from_xml_path(str(web/"public/robot/vector2/vector-free-shelf.xml"))
d = mujoco.MjData(m)
cal = json.loads((web/"camera_calibration_vector2.json").read_text())
focal = 300/np.tan(np.deg2rad(cal["reported_fov_y_deg"])/2)
corners = np.array(list(itertools.product([-.022,.022],repeat=3)))
rows = []
def xyz(p):
    return .001*np.array([p.get(k,0.) for k in "xyz"])
def rot(p):
    return Rotation.from_quat([p.get(k,0.) for k in ("q1","q2","q3","q0")])

for obs in json.loads((here/"aligned-images.json").read_text()):
    state, cube, box = obs["state"], obs["cube"], obs["img_rect"]
    measured = np.array([box["x_top_left"],box["y_top_left"]])
    size = np.array([box["width"],box["height"]])
    d.joint("root").qpos[:3] = xyz(state["pose"])
    d.joint("head_joint").qpos[0] = state["head_angle_rad"]
    for add_pitch in (False,True):
        chassis = rot(state["pose"])
        if add_pitch:
            chassis *= Rotation.from_rotvec([0,-state["pose_pitch_rad"],0])
        d.joint("root").qpos[3:] = chassis.as_quat(scalar_first=True)
        mujoco.mj_forward(m,d)
        camera = d.camera("head_cam")
        points = (xyz(cube)+rot(cube).apply(corners)-camera.xpos) @ camera.xmat.reshape(3,3)
        assert np.all(points[:,2]<0)
        projected = points[:,:2]/-points[:,2,None]
        for mode,scale,center in [("centered",[focal,-focal],[400,300]),
                                  ("calibrated",[cal["fx_px"],-cal["fy_px"]],[cal["cx_px"],cal["cy_px"]])]:
            uv = projected*scale + center
            lo,hi = uv.min(axis=0),uv.max(axis=0)
            area = np.maximum(0,np.minimum(measured+size,hi)-np.maximum(measured,lo)).prod()
            iou = area/(size.prod()+(hi-lo).prod()-area)
            delta = (lo+hi)/2-(measured+size/2)
            rows.append((obs["sec"],add_pitch,mode,iou,*delta,*(hi-lo-size)))
with (here/"camera-transfer.csv").open("w") as f:
    writer=csv.writer(f,lineterminator="\n")
    writer.writerow(["sec","extra_pitch","intrinsics","iou","dx_px","dy_px","dwidth_px","dheight_px"])
    writer.writerows(rows)
fig,axes=plt.subplots(1,2,figsize=(9,3.6),layout="constrained")
for mode in ("centered","calibrated"):
    r=[row for row in rows if row[2]==mode and not row[1]]
    t,_,_,iou,dx,dy,dw,dh=zip(*r)
    axes[0].plot(t,np.hypot(dx,dy),"o-",label=mode)
    axes[1].plot(t,iou,"o-",label=mode)
    print(mode,np.mean(np.hypot(dx,dy)),np.mean(iou),np.mean(np.abs(dw)),np.mean(np.abs(dh)))
for ax,title in zip(axes,("Cube centre error (pixels)","Cube rectangle overlap (IoU)")):
    ax.set(xlabel="Seconds from wheelstand request",title=title)
    ax.legend()
fig.savefig(here/"camera-transfer.png",dpi=170)
