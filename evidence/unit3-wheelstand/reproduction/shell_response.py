"""Isolate the free cube's shell contact time constant in the driven-support replay."""
import csv
import sys
from pathlib import Path

import replay

here = Path(__file__).resolve().parent
v = replay.params.values(replay.params.load(model="vector_2"), {"tip_supports_driven": 1})
with (here/"shell-response.csv").open("w", newline="") as f:
    w = csv.writer(f, lineterminator="\n")
    w.writerow(["run","case","shell_ms","peak_pitch_deg","hold_pitch_deg","pitch_rmse_deg","cube_peak_g","packet_changes"])
    for run in ("W4","W8"):
        tr = replay.Trial(run)
        for tau in (.004,.020):
            m = replay.build(v, sys.argv[1])
            m.opt.timestep = v["sim_timestep_ms"]*.001/4
            shell = [m.geom(i) for i in range(m.ngeom) if m.geom(i).name.startswith("cube_shell_")]
            assert shell
            for geom in shell:
                assert abs(geom.solref[0]-0.004)<1e-8
                geom.solref[0] = tau
            for i,(start,onset) in enumerate(((s+tr.shift,o) for s in (3.5,3.8) for o in (tr.early,tr.late)),1):
                events = []
                trace, pk, n, _ = replay.run(m,v,tr,start,onset,"servo:60",17,tap_times=events)
                out = replay.score(tr,trace,pk,n,onset)
                t = tr.cube["sec"]-tr.zero
                hi = min(tr.late+1.2,t[-1])
                w.writerow([run,i,tau*1000,out["peak_pitch_deg"],out["hold_pitch_deg"],out["pitch_rmse_deg"],out["peak_g"],
                            sum(tr.late-.3 <= t <= hi for t in events)])
