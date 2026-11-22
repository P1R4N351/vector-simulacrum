"""Map first-contact clearance in the plane of an upright, stationary W20 cube."""
import csv
import sys

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from free_departure import here, path, poses

model = mj.MjModel.from_xml_path(sys.argv[1])
cube = model.body("cube").id
fork = model.body("fork").id
side = np.arange(-36., 36.1, 1.5)
times = (5.239010112762716, 5.27, 5.33)
views = poses[poses["run"]=="W20"][-2:]
def wrist_pose(t):
    x,y,yaw,pitch,h = [np.interp(t,path["sec"],path[k]) for k in ("x","y","yaw","pitch","lift")]
    reach = -41+np.sqrt(66**2-(h-45)**2)
    along = -48+(reach+48)*np.cos(pitch)-(h-10.75)*np.sin(pitch)
    height = 10.75+(reach+48)*np.sin(pitch)+(h-10.75)*np.cos(pitch)
    pos = .001*np.array([x+along*np.cos(yaw),y+along*np.sin(yaw),height])
    q = Rotation.from_euler("ZY",[yaw,-pitch]).as_quat()
    return pos, q[[3,0,1,2]]

fig, axes = plt.subplots(2,3,figsize=(10,6.4),layout="constrained",sharex=True,sharey=True)
with (here/"contact-clearance.csv").open("w") as f:
    writer = csv.writer(f,lineterminator="\n")
    writer.writerow("pose time_s forward_mm left_mm min_contact_mm".split())
    for row,p in enumerate(views):
        forward = np.array([np.cos(p["robot_yaw"]),np.sin(p["robot_yaw"])])
        basis = np.stack([forward,[-forward[1],forward[0]]])
        center = np.array([p["cube_x_mm"],p["cube_y_mm"]])
        other = views[1-row]
        alt = basis @ (np.array([other["cube_x_mm"],other["cube_y_mm"]])-center)
        d = mj.MjData(model)
        d.qpos[2] = .02215
        d.qpos[3:7] = [np.cos(p["cube_yaw"]/2),0,0,np.sin(p["cube_yaw"]/2)]
        for col,t in enumerate(times):
            d.mocap_pos[0], d.mocap_quat[0] = wrist_pose(t)
            depths = np.zeros((len(side),len(side)))
            for i,fw in enumerate(side):
                for j,left in enumerate(side):
                    d.qpos[:2] = .001*(center+np.array([fw,left])@basis)
                    mj.mj_fwdPosition(model,d)
                    for c in d.contact:
                        a,b = model.geom_bodyid[c.geom]
                        if (a==cube and b==fork) or (a==fork and b==cube):
                            depths[i,j] = min(depths[i,j],c.dist*1000)
                    writer.writerow([row+1,round(t,6),fw,left,round(depths[i,j],6)])
            idx = np.argwhere(depths==0)
            closest = min((np.hypot(side[i],side[j]),side[i],side[j]) for i,j in idx)
            print(row+1,round(t,3),"nearest sampled clearance (norm, forward, left)",closest,flush=True)
            ax=axes[row,col]
            ax.contourf(side,side,(depths<0).astype(int),levels=[-.5,.5,1.5],colors=["#f7f5ef","#a0bec9"])
            ax.plot(0,0,"k+",ms=10)
            ax.plot(alt[1],alt[0],"kx",ms=7)
            ax.plot(closest[2],closest[1],"o",mfc="none",mec="darkred")
            ax.set_aspect("equal")
            if row==0:
                ax.set_title(f"{t:.3f} s")
            if col==0:
                ax.set_ylabel(("Earlier" if row==0 else "Final")+" sighting\nForward shift (mm)")
            if row==1:
                ax.set_xlabel("Leftward shift (mm)")
fig.suptitle("Blue: fork intersects cube     + recorded center     × alternate center     ○ nearest clear sample")
fig.savefig(here/"contact-clearance.png",dpi=160)

# Reject obvious intersections at three snapshots first, then test each millisecond.
checks = [wrist_pose(t) for t in (*times,*np.arange(5.205,5.331,.001))]
with (here/"clearance-through-descent.csv").open("w") as f:
    writer = csv.writer(f,lineterminator="\n")
    writer.writerow("pose forward_mm left_mm radial_mm smaller_grid_offsets_rejected".split())
    for row,p in enumerate(views):
        d = mj.MjData(model)
        center = np.array([p["cube_x_mm"],p["cube_y_mm"]])
        yaw = p["robot_yaw"]
        basis = np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
        d.qpos[2:7] = [.02215,np.cos(p["cube_yaw"]/2),0,0,np.sin(p["cube_yaw"]/2)]
        candidates = sorted((np.hypot(x,y),x,y) for x in side for y in side)
        for dist,x,y in candidates:
            d.qpos[:2] = .001*(center+np.array([x,y])@basis)
            for pos,quat in checks:
                d.mocap_pos[0],d.mocap_quat[0] = pos,quat
                mj.mj_fwdPosition(model,d)
                if any(c.dist<0 and set(model.geom_bodyid[c.geom])=={cube,fork} for c in d.contact):
                    break
            else:
                writer.writerow([row+1,x,y,dist,sum(item[0]<dist for item in candidates)])
                print("Clear through descent",row+1,x,y,dist,flush=True)
                break
