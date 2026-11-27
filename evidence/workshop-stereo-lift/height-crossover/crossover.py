"""Compare contiguous low/high pairs, preserving both orders of exposure."""
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
commands=[r for r in sdk if r["event"]=="command"]
scores=rows("summary.csv")
assert len(commands)==16 and len(scores)==8
with open("crossover.csv","w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["pair","trial","assigned_mm","measured_start_mm","in_position_s","encoder_rise_mm","left_rise_px","right_rise_px"])
    for i,r in enumerate(scores):
        prior,cmd=commands[2*i:2*i+2]
        t=float(cmd["relative_s"])
        before=[float(s["lift_mm"]) for s in states if t-.2<=float(s["relative_s"])<t]
        assert len(before)>=2 and float(cmd["target"])==52
        w.writerow([1+i//2,1+i,prior["target"],np.median(before),r["first_flag_s"],r["encoder_recovery_mm"],r["left_rise_px"],r["right_rise_px"]])
scores=np.genfromtxt("crossover.csv",delimiter=",",names=True)
for field in scores.dtype.names[3:]:
    diff=[]
    for pair in range(1,5):
        g=scores[scores["pair"]==pair]
        diff.append(g[field][g["assigned_mm"]==65][0]-g[field][g["assigned_mm"]==62.5][0])
    print(field,"high minus low",np.round(diff,3),"mean",np.mean(diff))
