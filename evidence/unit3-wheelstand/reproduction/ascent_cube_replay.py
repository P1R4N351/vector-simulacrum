"""Cube top-centre travel over intervals bracketed by W8 movie frames."""
import csv
import sys
import numpy as np
from wheel_authority import here, replay, servo


def main(fixture):
    tr = replay.Trial("W8")
    v = replay.params.values(replay.params.load(model="vector_2"), {"tip_supports_driven":1})
    rows = []
    for start in (3.5,3.8):
        m = replay.build(v,fixture)
        m.opt.timestep = .00025
        a = m.actuator("lift")
        a.gainprm[0],a.biasprm[1],a.biasprm[2] = 5,-5,-.03
        trace,_,_ = servo(m,v,tr,start,float(m.actuator("left_wheel").gainprm[0]),1.5)
        for first,last in ((-.136325,.103675),(.103675,.303675),(.303675,.623675)):
            seg = trace[(trace[:,0]-tr.late>=first)&(trace[:,0]-tr.late<=last)]
            rows.append([start,first,last,np.ptp(seg[:,2]),np.ptp(seg[:,3]),np.ptp(seg[:,1])])
    with (here/"ascent-cube-replay.csv").open("w") as f:
        w = csv.writer(f,lineterminator="\n")
        w.writerow("release first_s last_s top_radial_range_mm yaw_range_deg chassis_pitch_range_deg".split())
        w.writerows(rows)


if __name__=="__main__": main(sys.argv[1])
