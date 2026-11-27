"""Replay W8 at the original camera estimate and its published axis extremes."""
import csv
from pathlib import Path
import sys

import numpy as np

import replay
from wheelstand_load import FirstLoads

HERE = Path(__file__).resolve().parent


def main(fixture):
    tr = replay.Trial("W8")
    with (tr.dir/"camera-pose.csv").open() as f:
        rows = list(csv.DictReader(f))
    nominal = next(r for r in rows if r["case"]=="geometry" and [float(r[k]) for k in ("focal_px","lens_height_mm","fork_height_mm")]==[498.4,65.5,93])
    choices = {"nominal":nominal}
    for axis in ("forward_mm","left_mm"):
        choices["min "+axis] = min(rows,key=lambda r:float(r[axis]))
        choices["max "+axis] = max(rows,key=lambda r:float(r[axis]))
    far = choices["max forward_mm"]
    for axis in ("forward_mm","left_mm"):
        choices["isolated far "+axis] = dict(nominal,case="axis control",**{axis:far[axis]})
    xy = tr.cube_xy.copy()
    yaw = tr.obs(3.065,"yaw")
    rotation = np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    v = replay.params.values(replay.params.load(model="vector_2"),{"tip_supports_driven":1})
    m = replay.build(v,fixture)
    m.opt.timestep = v["sim_timestep_ms"]*.001/4
    with (HERE/"contact-registration.csv").open("w",newline="") as f:
        w = csv.writer(f,lineterminator="\n")
        w.writerow(["pose","source","forward_mm","left_mm","case","box_minus_fork_ms","held_pitch_deg","pitch_rmse_deg","max_initial_overlap_mm"])
        for label,row in choices.items():
            shift = np.array([float(row[k])-float(nominal[k]) for k in ("forward_mm","left_mm")])
            tr.cube_xy = xy + rotation@shift
            for case,(s,o) in enumerate(((s,o) for s in (3.5,3.8) for o in (tr.early,tr.late)),1):
                loads = FirstLoads(m)
                trace,pk,n,overlap = replay.run(m,v,tr,s+tr.shift,o,"servo:60",17,on_step=loads.step)
                score = replay.score(tr,trace,pk,n,o)
                w.writerow([label,row["case"],row["forward_mm"],row["left_mm"],case,1000*(loads.times.get("box",np.nan)-loads.times.get("fork",np.nan)),score["hold_pitch_deg"],score["pitch_rmse_deg"],overlap])


if __name__=="__main__":
    main(sys.argv[1])
