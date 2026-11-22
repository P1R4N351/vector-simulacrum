"""Estimate printed-fork occlusion and frame clipping of the camera-facing side glyph."""
import csv
from pathlib import Path
import sys
import json
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

here=Path(__file__).resolve().parent
root=here.parents[2]
sys.path.insert(0,str(root))
from simulacrum import mjcf, params

v=params.values(params.load(model="vector_2"))
cal=json.loads((root/"web/camera_calibration_vector2.json").read_text())
scene=ET.parse(root/"web/public/robot/vector2/vector-free-shelf.xml")
fixture=ET.parse(sys.argv[1])
scene.find("asset").extend(fixture.findall(".//asset/mesh"))
fork=scene.find(".//body[@name='lift_fork']")
for piece in fixture.findall(".//body[@name='fork']/geom[@type='mesh']"):
    ET.SubElement(fork,"geom",type="mesh",mesh=piece.get("mesh"),group="4",mass="0",contype="0",conaffinity="0")
assert len(fork.findall("geom[@group='4']"))==58
model=mj.MjModel.from_xml_string(ET.tostring(scene.getroot(),encoding="unicode"))
data=mj.MjData(model)
fork_only=np.array([0,0,0,0,1,0],np.uint8)
hit=np.zeros(1,np.int32)
def position(row,p):
    return np.array([row[p+k] for k in "xyz"])*.001
def quat(row,p):
    return np.array([row[p+"q"+str(i)] for i in range(4)])

fig,axes=plt.subplots(1,3,figsize=(12,3.8),layout="constrained",sharey=True)
with (here/"visibility.csv").open("w") as f:
    writer=csv.writer(f,lineterminator="\n")
    writer.writerow(["trial","sec","stamp","seen","pose_age_ms","lift_mm","size_mm","in_frame_fraction","visible_fraction"])
    for trial,ax in zip(("W4","W8","W12"),axes):
        results=[]
        for row in np.genfromtxt(here/f"{trial}.csv.gz",names=True,delimiter=","):
            data.joint("root").qpos[:3]=position(row,"r")
            data.joint("root").qpos[3:]=quat(row,"r")
            data.joint("head_joint").qpos[0]=row["head"]
            lift=mjcf.lift_angle_of_height(v,row["lift"])
            data.joint("lift_joint").qpos[0]=lift
            data.joint("wrist_joint").qpos[0]=-lift
            mj.mj_forward(model,data)
            cam=data.camera("head_cam")
            center=position(row,"c")
            rotation=Rotation.from_quat(quat(row,"c"),scalar_first=True)
            local_eye=rotation.inv().apply(cam.xpos-center)
            normal=int(np.argmax(np.abs(local_eye[:2])))
            for width in (20,23):
                grid=np.linspace(-width/2,width/2,17)*.001
                points=np.zeros((17*17,3))
                points[:,normal]=np.sign(local_eye[normal])*.022
                points[:,[1-normal,2]]=np.array(np.meshgrid(grid,grid)).reshape(2,-1).T
                rays=rotation.apply(points)+center-cam.xpos
                view=rays@cam.xmat.reshape(3,3)
                pixels=view[:,:2]/-view[:,2,None]*[cal["fx_px"],-cal["fy_px"]]+[cal["cx_px"],cal["cy_px"]]
                inside=(view[:,2]<0)&(pixels>=0).all(axis=1)&(pixels<[800,600]).all(axis=1)
                visible=0
                for ray in rays[inside]:
                    distance=np.linalg.norm(ray)
                    depth=mj.mj_ray(model,data,cam.xpos,ray/distance,fork_only,1,-1,hit)
                    visible+=int(depth<0 or depth>=distance)
                vals=[row["sec"],int(row["stamp"]),int(row["seen"]),int(row["age_ms"]),row["lift"],width,inside.mean(),visible/len(rays)]
                writer.writerow([trial,*vals])
                if width==23:results.append(vals)
        s,stamp,seen,age,lift,w,inside,clear=np.array(results).T
        ax.plot(s,inside,"--",label="Within image")
        ax.plot(s,clear,label="Within image and clear of fork")
        ax.scatter(s[seen==1],clear[seen==1],color="black",s=18,label="SDK cube sighting",zorder=3)
        ax.set(xlabel="Seconds from request",title=trial,ylim=(-.05,1.05))
        ax.grid(alpha=.2)
axes[0].set_ylabel("Fraction of facing marker")
axes[-1].legend(fontsize=7,loc="lower left")
fig.savefig(here/"visibility.png",dpi=170)
