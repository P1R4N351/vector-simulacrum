"""Change rear tip friction alone and measure loaded contact slip during ascent."""
import csv
import sys
from unittest.mock import patch

import matplotlib.pyplot as plt
import numpy as np

from wheel_authority import here, replay, servo


def measure(v,fixture,tr,start,mu):
    model = replay.build(v,fixture)
    model.opt.timestep = .00025
    lift = model.actuator("lift")
    lift.gainprm[0], lift.biasprm[1], lift.biasprm[2] = 5,-5,-.03
    stall = float(model.actuator("left_wheel").gainprm[0])
    ids = [model.geom("tip_rear_"+s).id for s in "lr"]
    model.geom_friction[ids,0] = mu
    sample = replay._Run.sample
    records = []
    wrench = np.empty(6)
    jac = np.empty((3,model.nv))
    partners = set()

    def watch(r,i,t):
        sample(r,i,t)
        if not tr.late-.01 <= t <= tr.late+.81: return
        axis = np.array([np.cos(r.yaw),np.sin(r.yaw),0])
        axle = np.mean(r.d.geom_xpos[ids],axis=0) @ axis
        load = weighted = 0.
        for j,c in enumerate(r.d.contact):
            a,b = c.geom
            if a not in ids and b not in ids: continue
            tip, other = (a,b) if a in ids else (b,a)
            replay.mj.mj_contactForce(model,r.d,j,wrench)
            if wrench[0]<=0: continue
            partners.add(model.geom(other).name)
            assert model.geom_bodyid[other]==0 and np.isclose(c.friction[0],mu)
            replay.mj.mj_jac(model,r.d,jac,None,c.pos,model.geom_bodyid[tip])
            speed = axis @ (jac @ r.d.qvel)
            load += wrench[0]
            weighted += wrench[0]*abs(speed)
        records.append([t,axle,load,weighted])

    with patch.object(replay._Run,"sample",watch):
        trace,w,score = servo(model,v,tr,start,stall,1.5)
    d = np.array(records)
    early = (d[:,0]>=tr.late)&(d[:,0]<=tr.late+.275)
    impulse = np.sum(d[early,2])*.001
    slip = 1000*d[early,3].sum()/d[early,2].sum()
    travel = 1000*np.diff(np.interp([tr.late,tr.late+.275],d[:,0],d[:,1]))[0]
    t = tr.state["sec"]-tr.zero
    valid = (t>=tr.late)&(t<=tr.late+.8)
    err = [np.interp(t[valid],w[:,0],w[:,i])-tr.state[s][valid] for i,s in ((1,"left"),(3,"right"))]
    return [travel,slip,impulse,np.sqrt(np.mean(np.square(err))),*score[:3],",".join(sorted(partners))]


def main(fixture):
    rows=[]
    fig,axes=plt.subplots(2,3,figsize=(10,6),layout="constrained",sharex=True)
    v=replay.params.values(replay.params.load(model="vector_2"),{"tip_supports_driven":1})
    for axes_row,name in zip(axes,("W4","W8")):
        tr=replay.Trial(name)
        for start in (3.5,3.8):
            points=[]
            for mu in (.3,v["tread_friction"],1.5):
                result=measure(v,fixture,tr,start,mu)
                rows.append([name,start,mu,*result])
                points.append([mu,*result[:-1]])
            points=np.array(points)
            for ax,k,label in zip(axes_row,(2,1,6),("Loaded slip speed (mm/s)","Rear axle travel (mm)","Peak pitch (°)")):
                ax.plot(points[:,0],points[:,k],"o-",label=f"Release {start} s")
                ax.set_title(name+" · "+label,fontsize=10)
        axes_row[-1].axhline(tr.physical()["peak_pitch_deg"],ls="--",color="k",label="Physical peak")
    axes[0,-1].legend(fontsize=7)
    for ax in axes[-1]: ax.set_xlabel("Rear support friction coefficient")
    fig.savefig(here/"rear-grip.png",dpi=160)
    with (here/"rear-grip.csv").open("w") as f:
        writer=csv.writer(f,lineterminator="\n")
        writer.writerow("run release mu axle_mm slip_mmps normal_impulse_Ns wheel_rmse_mmps lift_rmse_mm peak_deg held_deg partners".split())
        writer.writerows(rows)


if __name__=="__main__": main(sys.argv[1])
