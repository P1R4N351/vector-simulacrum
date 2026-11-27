import csv
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory
import numpy as np

here=Path(__file__).resolve().parent
runner=here.parents[1]/"workshop-joints-20261127/lift-duration/replay.mjs"
with (here/"sdk.csv").open() as f:
    rows=list(csv.DictReader(f))
with TemporaryDirectory() as folder:
    path=Path(folder)
    with (path/"recording.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["s","event","target_mm","duration_s","lift_mm","head_deg","gy","left","right","dock","code","in_position"])
        for r in rows:
            w.writerow([r["relative_s"],r["event"],r["target"],r["duration_s"],r["lift_mm"],r["head_deg"],r["gy"],r["left"],r["right"],r["dock"],"",r["in_position"]])
    subprocess.run(["node",str(runner),path.as_uri()+"/"],check=True)
    sim=np.genfromtxt(path/"replay.csv",delimiter=",",names=True)
cmd=[r for r in rows if r["event"]=="command"]
with (here/"sim.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n");w.writerow(["trial","prep_mm","sim_start_mm","sim_first_flag_s","sim_recovery_mm"])
    for i in range(1,len(cmd),2):
        t=float(cmd[i]["relative_s"]); first=np.flatnonzero((sim["s"]>=t)&(sim["in_position"]==0))[0]
        k=first+np.flatnonzero(sim["in_position"][first:]==1)[0]
        w.writerow([1+i//2,cmd[i-1]["target"],np.median(sim["lift_mm"][(sim["s"]>=t-.2)&(sim["s"]<t)]),sim["s"][k]-t,sim["lift_mm"][k]-np.min(sim["lift_mm"][first:k+1])])
