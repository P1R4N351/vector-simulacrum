"""Choose a contact on the early pickup frame, then freeze it for five independent pushes."""
from collections import defaultdict
import csv
import importlib
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import matplotlib.pyplot as plt
import numpy as np

here=Path(__file__).resolve().parent
source=Path(sys.argv[1]).resolve()
repo=Path(sys.argv[2]).resolve()
variant=importlib.import_module("shelf-corner-replay").shelf_variant
with (here/"shelf-corner-replay.csv").open() as f:
    rows=list(csv.DictReader(f))
def score(r):
    return np.hypot((float(r["slide_2p59_mm"])-19.268)/19.268,(float(r["yaw_2p59_deg"])+7.611)/7.611)
errors=defaultdict(list)
for r in rows:
    if r["mu"]!="unsplit":
        errors[float(r["mu"]),float(r["pad_width_fraction"])].append(score(r))
winner=min(errors,key=lambda k:max(errors[k]))
print("Selected",winner,max(errors[winner]))
sys.path.insert(0,str(repo/"evidence/unit3-printed-fork-contact"))
sys.argv[1]=str(repo)
import evaluate
out=[]
with TemporaryDirectory() as folder:
    for label,settings in [("canonical",None),("split control",(.9,1)),("selected",winner)]:
        fixture=source
        if settings:
            fixture=Path(folder)/"floor.xml"
            variant(source,*settings).write(fixture)
        sys.argv[2]=str(fixture)
        for trial in range(1,6):
            s,trace=evaluate.run((trial,"stl","overhead","weld",0,0,0),rails=True)
            out.append((label,trial,s[10],s[11],s[12],abs((s[10]-s[11]+180)%360-180)))
with (here/"shelf-corner-transfer.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow("model trial replay_yaw_deg camera_yaw_deg xy_error_mm yaw_error_deg".split())
    w.writerows(out)
fig,axes=plt.subplots(2,2,figsize=(9,6.2),layout="constrained")
for step in [1,.5]:
    r=sorted([r for r in rows if r["mu"]!="unsplit" and float(r["dt_ms"])==step and float(r["pad_width_fraction"])==1],key=lambda r:float(r["mu"]))
    for ax,key,obs in zip(axes[0],["slide_2p86_mm","yaw_2p86_deg"],[27.199,-13.591]):
        ax.plot([float(x["mu"]) for x in r],[float(x[key]) for x in r],"o-" if step==1 else "x--",label=f"{step:g} ms")
        if step==1: ax.axhline(obs,ls=":",color="k",label="Camera")
for ax,title in zip(axes[0],["Travel (mm)","Yaw (°, CCW positive)"]):
    ax.set(xlabel="Shelf–rubber sliding friction",ylabel=title,title="Pickup at +2.86 s, full pad width")
axes[0,0].legend(fontsize=8)
for label in ("canonical","selected"):
    r=[x for x in out if x[0]==label]
    for ax,i,y in zip(axes[1],[4,5],["Endpoint error (mm)","Yaw error (°)"]):
        ax.plot([x[1] for x in r],[x[i] for x in r],"o-",label=label)
        ax.set(xlabel="Independent push trial",ylabel=y,xticks=range(1,6))
axes[1,0].legend(fontsize=8)
fig.savefig(here/"shelf-corner.png",dpi=155)
