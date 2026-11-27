"""Measure W8 fork/cube signed clearance with simulated and encoder-driven joint poses."""
import csv
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np

import replay

HERE = Path(__file__).resolve().parent
POSES = ("early","square XY and yaw","feature0 axis0 +2px")


class Clearance:
    def __init__(self,m):
        self.m = m
        self.d = mj.MjData(m)
        self.fork = np.flatnonzero((m.geom_bodyid==m.body("lift_fork").id)&(m.geom_type==mj.mjtGeom.mjGEOM_MESH))
        self.cube = np.flatnonzero(m.geom_bodyid==m.body("cube").id)
        assert len(self.fork)==58 and len(self.cube)>0
        self.radii = m.geom_rbound[self.fork,None]+m.geom_rbound[self.cube]

    def closest(self, exhaustive=False):
        m,d = self.m,self.d
        mj.mj_forward(m,d)
        lower = np.linalg.norm(d.geom_xpos[self.fork,None]-d.geom_xpos[self.cube],axis=2)-self.radii
        best = .15
        pair = None
        # Bounding spheres supply a conservative lower bound for outside distances.
        for flat in np.argsort(lower,axis=None):
            i,j = np.unravel_index(flat,lower.shape)
            if not exhaustive and lower[i,j]>max(best,0):
                break
            a,b = self.fork[i],self.cube[j]
            dist = mj.mj_geomDistance(m,d,a,b,.15,None)
            if dist<best:
                best,pair = dist,(a,b)
        assert pair is not None
        witness=np.empty(6)
        check=mj.mj_geomDistance(m,d,*pair,.15,witness)
        assert abs(check-best)<1e-8
        frame=d.xmat[m.jnt_bodyid[m.joint("root").id]].reshape(3,3)
        local_mm=frame.T@(witness[3:]-witness[:3])*1000
        if best>0:
            assert abs(np.linalg.norm(local_mm)-1000*best)<1e-5
        return best*1000,m.geom(pair[0]).name,m.geom(pair[1]).name,local_mm


def main(fixture):
    tr = replay.Trial("W8")
    v = replay.params.values(replay.params.load(model="vector_2"),{"tip_supports_driven":1})
    m = replay.build(v,fixture)
    m.opt.timestep=v["sim_timestep_ms"]*.001/4
    reader=Clearance(m)
    with (HERE/"early-pose.csv").open() as f:
        poses={r["estimate"]:r for r in csv.DictReader(f)}
    t0=.625
    yaw=tr.obs(t0,"yaw")
    co,si=np.cos(yaw),np.sin(yaw)
    transform=np.array([[co,-si],[si,co]])
    origin=np.array([tr.obs(t0,k) for k in ("x_mm","y_mm")])
    fig,axs=plt.subplots(2,1,layout="constrained",figsize=(9,6),sharex=True)
    adr=int(m.joint("cube_root").qposadr[0])
    times=np.arange(1.20,1.701,.005)
    with (HERE/"fork-clearance.csv").open("w",newline="") as f:
        out=csv.writer(f,lineterminator="\n")
        out.writerow(["pose","joint_source","time_s","lift_mm","signed_gap_mm","fork_geom","cube_geom","fork_to_cube_fwd_mm","fork_to_cube_left_mm","fork_to_cube_up_mm"])
        for idx,label in enumerate(POSES):
            p=poses[label]
            tr.cube_xy=origin+transform@np.array([float(p["forward_mm"]),float(p["left_mm"])])
            tr.cube_yaw=np.deg2rad(float(p["cube_yaw_deg"]))
            heights=[]; gaps=[]
            def step(run,t):
                i=len(gaps)
                if i>=len(times) or t<times[i]-1e-8:
                    return
                d=reader.d
                d.qpos[:]=run.d.qpos
                # Sample the swept geometry against the original upright cube, even after possible contact.
                d.qpos[adr:adr+2]=tr.cube_xy*.001
                d.qpos[adr+2]=m.qpos0[adr+2]
                d.qpos[adr+3:adr+7]=[np.cos(tr.cube_yaw/2),0,0,np.sin(tr.cube_yaw/2)]
                servo=d.joint("lift_joint").qpos[0]
                measured=tr.obs(t,"lift_mm")
                h=replay.mjcf.lift_height_of_angle(v,servo)
                heights.append([t,h,measured])
                row=[]
                for mode,angle in (("servo",servo),("servo level wrist",servo),("encoder level wrist",replay.mjcf.lift_angle_of_height(v,measured))):
                    d.joint("lift_joint").qpos[0]=angle
                    if mode!="servo":
                        d.joint("wrist_joint").qpos[0]=-angle
                    gap,fork,cube,delta=reader.closest()
                    if i==55:  # 1.475 s, the encoder-driven nearest approach in these poses
                        full,*_ = reader.closest(exhaustive=True)
                        assert abs(full-gap)<1e-6
                    row.append(gap)
                    out.writerow([label,mode,t,replay.mjcf.lift_height_of_angle(v,angle),gap,fork,cube,*delta])
                gaps.append(row)
            replay.run(m,v,tr,t0,tr.early,"servo:60",17,on_step=step)
            assert len(gaps)==len(times)
            h=np.array(heights)
            if idx==0:
                axs[0].plot(h[:,0],h[:,1],label="Finite actuator")
                ts=tr.approach["s"]
                sel=(ts>=times[0]-.05)&(ts<=times[-1]+.05)
                axs[0].plot(ts[sel],tr.approach["lift_mm"][sel],"ko-",ms=3,label="Encoder")
            a=np.array(gaps)
            axs[1].plot(h[:,0],a[:,0],ls=":",color=f"C{idx}",label=label+" · servo")
            axs[1].plot(h[:,0],a[:,2],color=f"C{idx}",label=label+" · encoder")
    axs[0].set_ylabel("Lift height (mm)")
    axs[1].set_ylabel("Fork–cube clearance (mm)")
    axs[1].set_xlabel("Seconds after wheelstand request")
    axs[1].axhline(0,color="black",lw=.7)
    for ax in axs:
        ax.legend(fontsize=7,ncol=2)
        ax.axvspan(1.35,1.58,color="grey",alpha=.12)
        ax.set_xlim(times[0],times[-1])
    fig.savefig(HERE/"fork-clearance.png",dpi=160)


if __name__=="__main__":
    main(sys.argv[1])
