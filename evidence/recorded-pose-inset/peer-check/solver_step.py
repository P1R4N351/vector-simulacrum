"""Timestep-only convergence at frozen 3.75 mm inset, stock and 0 mm proxy controls."""
import argparse
from collections import defaultdict
from contextlib import contextmanager
import csv
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
from simulacrum import mjcf, params, replay, recording
from fit_p2 import read_inputs, simulated
import c7_transfer


@contextmanager
def timestep_only(ms):
    """Keep the 1 ms model's SI coupling/contact time constants and change integration alone."""
    build = mjcf.build_mjcf
    def frozen_xml(values, *args, **kwargs):
        root = ET.fromstring(build(dict(values, sim_timestep_ms=1.), *args, **kwargs))
        root.find('option').set('timestep', str(ms/1000.))
        return ET.tostring(root, encoding='unicode')
    with patch.object(mjcf,'build_mjcf', frozen_xml):
        yield


def write(path, rows):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, list(rows[0])); writer.writeheader(); writer.writerows(rows)


def score_c7(m):
    return dict(yaw=m['mean_abs_yaw_rad_s'], roll=m['mean_abs_roll_rad_s'],
                pitch=m['mean_abs_pitch_rad_s'], center_mm=m['simulation_icr_x_mm'])


def record_rows(p2, c7):
    rows = []
    for m in p2:
        rows.append(dict(robot='P2Y8', spin=m['spin'], step_ms='', inset_mm='', **{k:m[k] for k in ('yaw','roll','pitch','center_mm')}))
    for m in c7:
        rows.append(dict(robot='C7K8', spin=m['spin'], step_ms='', inset_mm='', **score_c7(m)))
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('p2',type=Path); parser.add_argument('c7',type=Path)
    parser.add_argument('c7_scores',type=Path); parser.add_argument('out',type=Path)
    args=parser.parse_args()
    p2_runs, _, p2_truth=read_inputs(args.p2)
    with args.c7_scores.open() as f:
        c7_truth=[r for r in csv.DictReader(f) if r['source']=='C7K8']
    rows=record_rows(p2_truth,c7_truth)
    v1=params.values(params.load(model='vector_1'))
    states,commands=recording.load(args.c7)
    c7_runs=[s for s in recording.segments(commands,v1).values() if s.kind=='wheels' and s.left*s.right<0]
    for ms in (1.,.5,.25,.125):
        for inset in (-1.,0.,3.75):
            with timestep_only(ms):
                for seg,start,t,v2 in p2_runs:
                    _,m=simulated(inset,seg,start,t,dict(v2,sim_timestep_ms=ms))
                    rows.append(dict(robot='P2Y8',spin=seg.label,step_ms=ms,inset_mm=inset,**{k:m[k] for k in ('yaw','roll','pitch','center_mm')}))
                for seg in c7_runs:
                    trace=c7_transfer.simulate(dict(v1,sim_timestep_ms=ms),replay._state_at(states,seg.t_cmd),seg,None if inset<0 else inset)
                    score=c7_transfer.score(trace,seg.t_stop-seg.t_cmd,'sim')
                    rows.append(dict(robot='C7K8',spin=seg.label,step_ms=ms,inset_mm=inset,**score_c7(score)))
    write(args.out/'solver_step.csv',rows)
    groups=defaultdict(list)
    truth={(r['robot'],r['spin']):r for r in rows if r['step_ms']==''}
    for r in rows:
        if r['step_ms']!='': groups[r['robot'],r['step_ms'],r['inset_mm']].append(r)
    summary=[]
    for (robot,ms,inset),group in groups.items():
        d=dict(robot=robot,step_ms=ms,inset_mm=inset)
        for k in ('yaw','roll','pitch','center_mm'):
            if robot=='C7K8' and k=='center_mm':
                d[k+'_mean']=float(np.mean([r[k] for r in group]))
                d[k+'_mae']=''
                continue
            actual=np.array([float(truth[robot,r['spin']][k]) for r in group])
            values=np.array([r[k] for r in group])
            d[k+'_mean']=float(values.mean())
            d[k+'_mae']=float(np.mean(abs(values-actual)))
            if k=='yaw': d['yaw_relative_rms']=float(np.sqrt(np.mean(((values-actual)/actual)**2)))
        summary.append(d)
    write(args.out/'solver_step_summary.csv',summary)

if __name__=='__main__': main()
