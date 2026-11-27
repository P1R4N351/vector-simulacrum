"""Infer animation phase from predeparture joint observations, holding out wheel and charger signals."""
import csv
import numpy as np
from compare import obs,clips,metrics,rows

with open("posture.csv","w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["trial","clip","lag_s","head_rmse_deg","lift_rmse_mm","wheel_rmse_mmps","edge_wheel_rmse_mmps","first_off_error_s"])
    for n in "ABCDEF":
        q=[r for r in rows(f"../unit3-physical-repeats/{n}/robot.csv.gz") if 0<=float(r["receipt_s"])<=6]
        t=np.array([float(x["receipt_s"]) for x in q])
        h=np.degrees([float(x["head_rad"]) for x in q]);l=np.array([float(x["lift_mm"]) for x in q])
        def loss(k,delay):
            s=clips[k]
            a=np.interp(t-delay,s["t"],s["head"],left=h[0])-h
            b=np.interp(t-delay,s["t"],s["lift"],left=l[0])-l
            # Equal weight after normalizing by physical joint excursion range.
            return np.mean((a/67)**2+(b/60)**2),np.mean(a*a)**.5,np.mean(b*b)**.5
        _,k,d,hr,lr=min((score,k,d,hr,lr) for k in clips if k!="canonical"
                 for d in np.arange(0,5.51,.05) for score,hr,lr in [loss(k,d)])
        w.writerow([n,k,d,hr,lr,*metrics(k,d,n)])
