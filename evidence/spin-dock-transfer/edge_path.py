"""Build offline shaft position knots at frame time and captured edge time."""
import csv
import numpy as np
from pathlib import Path
here=Path(__file__).resolve().parent
def load(name):
    return np.genfromtxt(here.parent/"hw-rec-20261119"/name,delimiter=",",names=True,dtype=None,encoding="utf8")
full=load("unit2-native-replay-input.csv")
native=load("unit2-native-stops.csv")
with (here/"edge-knots.csv").open("w",newline="") as f:
    w=csv.writer(f,lineterminator="\n"); w.writerow(["turn","side","scheme","ms","position_mm"])
    for turn in np.unique(full["turn"]):
        s=full[full["turn"]==turn]; n=native[native["turn"]==turn]
        for side,sign in [("left",1),("right",-1)]:
            p=side+"_count"; t=n["ms_from_last_edge"]
            idx=np.flatnonzero((t>=-100)&(t<=0)&(np.diff(n[p],prepend=n[p][0])!=0))
            intervals=n[side+"_ticks"][idx]*1000/(48e6*256)
            rel=np.cumsum(intervals); rel-=rel[-1]
            lo=max(t[idx-1]-rel)-.5; hi=min(t[idx]-rel)+.5
            assert lo<=hi
            cut=t[idx[0]-1]
            for scheme in ("frame","fresh","early","center","late"):
                if scheme=="frame":
                    times=s["ms_from_last_edge"]; counts=s[p]
                else:
                    pre=s["ms_from_last_edge"]<cut
                    last={"early":lo,"late":hi,"center":(lo+hi)/2,"fresh":0}[scheme]
                    times=np.r_[s["ms_from_last_edge"][pre],(t[idx] if scheme=="fresh" else rel+last),s["ms_from_last_edge"][-1]]
                    counts=np.r_[s[p][pre],n[p][idx],s[p][-1]]
                assert np.all(np.diff(times)>0)
                for ms,count in zip(times,counts):
                    w.writerow([turn,side,scheme,ms,sign*(count-s[p][0])*.96*29*.25*np.pi/172.3])
