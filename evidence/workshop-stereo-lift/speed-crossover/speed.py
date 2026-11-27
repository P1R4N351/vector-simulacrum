import csv
from pathlib import Path
import runpy
import numpy as np

runpy.run_path(str(Path(__file__).resolve().parent.parent/"recovery/analyze.py"))
def rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))
sdk=rows("sdk.csv")
states=[r for r in sdk if r["event"]=="state"]
commands=[r for r in sdk if r["event"]=="command" and r["target"]=="52"]
t=np.array([float(r["relative_s"]) for r in states])
h=np.array([float(r["lift_mm"]) for r in states])
def crossing(times,heights,threshold):
    k=np.flatnonzero(heights<=threshold)[0]
    assert k>0
    return times[k-1]+(times[k]-times[k-1])*(heights[k-1]-threshold)/(heights[k-1]-heights[k])
with open("speed-scores.csv","w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["trial","duration_s","pre_height_mm","speed20_80_mmps","ready_s","ready_minus_duration_s","recovery_mm","left_rise_px","right_rise_px"])
    for i,(c,r) in enumerate(zip(commands,rows("summary.csv"),strict=True)):
        start=float(c["relative_s"]);d=float(c["duration_s"])
        initial=np.median(h[(t>=start-.2)&(t<start)])
        m=(t>=start-.15)&(t<=start+float(r["first_flag_s"])+.1)
        stroke=initial-52
        t20,t80=[crossing(t[m],h[m],initial-p*stroke) for p in (.2,.8)]
        w.writerow([i+1,d,initial,.6*stroke/(t80-t20),r["first_flag_s"],float(r["first_flag_s"])-d,r["encoder_recovery_mm"],r["left_rise_px"],r["right_rise_px"]])
