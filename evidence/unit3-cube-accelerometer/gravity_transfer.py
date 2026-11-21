"""Check the upright gravity response candidates on the two held-out loaded turns."""
import csv
import numpy as np

from replay_turns import predicted, values
from static_gravity import actions, here, packets, zero

def main():
    rest=packets("resting")[1].mean(axis=0)
    z=np.sqrt(1-rest[:2]@rest[:2])
    candidates=[("Nominal",1.,0.,1.),("Common gain",np.linalg.norm(rest),0.,np.linalg.norm(rest)),
                ("Z offset",1.,rest[2]-z,1.),("Z gain",rest[2]/z,0.,1.)]
    start,back=[float(e["pod_unix_s"]) for e in actions if e["action"]=="turn loaded" and e["event"]=="begin"]
    end=next(float(e["pod_unix_s"]) for e in actions if e["action"]=="place")
    t,measured=packets("pickup")
    t-=start-zero
    static=np.median(measured[(t>=-1.7)&(t<=-.2)],axis=0)
    robot=np.genfromtxt(here/"robot.csv",names=True,delimiter=",")
    rt=robot["pod_unix_s"]-start
    use=(rt>-2.1)&(rt<end-start+.2)
    with (here/"gravity-transfer.csv").open("w") as out:
        w=csv.writer(out,lineterminator="\n")
        w.writerow(["pose","candidate","turn","vector_rms_g","z_rms_g","mean_z_error_g"])
        for pose in ("fixed","corrected"):
            for name,gain,bias,xy_gain in candidates:
                scale=np.array([xy_gain,xy_gain,gain])
                offset=np.array([0.,0.,bias])
                a=(static-offset)/scale if pose=="corrected" else static.copy()
                a/=np.linalg.norm(a)
                q=np.r_[1+a[2],np.cross(a,[0,0,1])]
                clock,p=predicted(rt[use],robot[use],q/np.linalg.norm(q),
                    dict(values,cube_accel_scale=scale,cube_accel_bias_mg=offset*1000))
                prediction=np.stack([np.interp(t,clock,col) for col in p.T],axis=1)
                for turn,left,right in [("outward",0,back-start),("return",back-start,end-start)]:
                    mask=(t>=left)&(t<right)
                    err=(prediction-measured)[mask]
                    w.writerow([pose,name,turn,*np.round([np.sqrt(np.mean((err**2).sum(axis=1))),
                                 np.sqrt(np.mean(err[:,2]**2)),np.mean(err[:,2])],5)])
if __name__=="__main__":
    main()
