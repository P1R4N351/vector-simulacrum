"""One-factor contact relaxation sweep with geometry and solver step held fixed."""
import csv
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
PARENT = HERE.parent
sys.path.insert(0, str(PARENT))
from fit_p2 import read_inputs, simulated
from c7_transfer import simulate, score
from simulacrum import params, recording, replay


def summarize(robot, tc, calc, obs):
    rate_error = np.asarray([r['yaw']/o['yaw']-1 for r,o in zip(calc,obs)])
    return dict(robot=robot, relaxation_ms=tc, rms_yaw_pct=np.sqrt(np.mean(rate_error**2))*100,
                roll=np.mean([r['roll'] for r in calc]), pitch=np.mean([r['pitch'] for r in calc]),
                center_mm=np.mean([r['center_mm'] for r in calc]))


def main():
    runs, _, obs = read_inputs(PARENT.parent/'camera-spins')
    results=[]
    for tc in [2,4,6,8,12,16]:
        scores=[simulated(3.75,seg,start,ts,dict(values,contact_time_constant_ms=tc))[1]
                for seg,start,ts,values in runs]
        results.append(summarize('P2Y8',tc,scores,obs))
    vals=params.values(params.load(model='vector_1'))
    src=PARENT.parent/'hw-rec-20261118/run-e-spins.jsonl'
    states, cmds=recording.load(src)
    with (PARENT/'c7_scores.csv').open() as f:
        observed={r['spin']:dict(yaw=float(r['mean_abs_yaw_rad_s'])) for r in csv.DictReader(f) if r['source']=='C7K8'}
    for tc in [2,4,6,8,12,16]:
        scores=[]; obs=[]
        for seg in recording.segments(cmds,vals).values():
            if seg.label not in observed:
                continue
            sim=simulate(dict(vals,contact_time_constant_ms=tc),replay._state_at(states,seg.t_cmd),seg,3.75)
            r=score(sim,seg.t_stop-seg.t_cmd,'model')
            scores.append(dict(yaw=r['mean_abs_yaw_rad_s'],roll=r['mean_abs_roll_rad_s'],
                               pitch=r['mean_abs_pitch_rad_s'],center_mm=r['simulation_icr_x_mm']))
            obs.append(observed[seg.label])
        results.append(summarize('C7K8',tc,scores,obs))
    with (HERE/'relaxation.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=results[0]); w.writeheader();w.writerows(results)


if __name__=='__main__':
    main()
