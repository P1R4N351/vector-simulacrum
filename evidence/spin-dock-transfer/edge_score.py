"""Score filtered gyro on native observation times, with no fitted alignment."""
from pathlib import Path
import csv
import sys
import matplotlib.pyplot as plt
import numpy as np

def read(p):
    return np.genfromtxt(p,names=True,delimiter=",",dtype=None,encoding="utf8")
h=read("../hw-rec-20261119/unit2-native-stops.csv")
prefix=sys.argv[1] if len(sys.argv)>1 else "edge"
r=read(prefix+"-replay.csv")
fig,axes=plt.subplots(2,2,figsize=(10,6),layout="constrained",sharex=True)
with open(prefix+"-scores.csv","w",newline="") as f:
    out=csv.writer(f,lineterminator="\n")
    out.writerow(["turn","scheme","braking_rms_dps","tail_rms_dps","steady_max_change_dps","final_travel_change_mm"])
    for turn,ax in zip(("ccw_1","cw_1","ccw_2","cw_2"),axes.flat):
        hw=h[h["turn"]==turn];t=hw["ms_from_last_edge"]; y=np.degrees(hw["head_gz"])
        show=(t>=-120)&(t<=100)
        ax.plot(t[show],y[show],"k.",label="Physical head gyro",ms=4)
        base=r[(r["turn"]==turn)&(r["scheme"]=="frame")]
        for scheme in ("frame","fresh","early","center","late"):
            s=r[(r["turn"]==turn)&(r["scheme"]==scheme)]
            pred=np.interp(t,s["ms"],s["head_gyro_dps"])
            def rms(a,b):
                mask=(t>=a)&(t<=b);return np.sqrt(np.mean((pred[mask]-y[mask])**2))
            steady=s["ms"] < -120
            out.writerow((turn,scheme,rms(-100,100),rms(0,100),max(abs(s["head_gyro_dps"][steady]-base["head_gyro_dps"][steady])),max(abs(s[k][-1]-base[k][-1]) for k in ("left_mm","right_mm"))))
            if scheme in ("frame","fresh","center"):
                ax.plot(t[show],pred[show],label=scheme)
        ax.set(title=turn.upper().replace("_"," "),xlabel="Time from final changing frame (ms)",ylabel="Head yaw rate (°/s)")
axes.flat[0].legend(fontsize=7)
fig.savefig(prefix+"-replay.png",dpi=165)
