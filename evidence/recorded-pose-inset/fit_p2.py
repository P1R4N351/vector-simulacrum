"""Fit longitudinal contact inset on P2Y8's four camera spins, excluding all other robots."""
import argparse
import csv
import json
import math
from pathlib import Path
from unittest.mock import patch
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from scipy.optimize import least_squares
from simulacrum import mjcf, params, recording, replay
from simulacrum.robot import VectorSim
from contact import contact_xml
import camera_spins as opt

COLUMNS = 'spin series t_s yaw roll pitch head_gz wl wr head_deg lift_mm x_mm y_mm heading_rad'.split()
def row(label, series, t, g, head, wl, wr, lift, x, y, yaw, off):
    a = head + math.radians(off)
    return dict(zip(COLUMNS, [label, series, t, g[0]*math.sin(a)+g[2]*math.cos(a),
        g[0]*math.cos(a)-g[2]*math.sin(a), g[1], g[2], wl, wr, math.degrees(head), lift, x,y,yaw]))

def metrics(rows, inset, center):
    steady = [r for r in rows if .7-1e-8 <= r['t_s'] <= 1.4+1e-8]
    return dict(spin=rows[0]['spin'],series=rows[0]['series'],inset_mm=inset,
                **{k: float(np.mean([abs(r[k]) for r in steady])) for k in ['yaw','roll','pitch','head_gz','wl','wr']},
                center_mm=center, n=len(steady))

def simulated(inset, seg, start, camera_times, values):
    kind = 'stock' if inset < 0 else f'proxy_{inset:g}'
    old = mjcf.build_mjcf
    def render(*args, **kwargs):
        xml = old(*args, **kwargs)
        return xml if inset<0 else contact_xml(xml,inset)
    with patch.object(mjcf,'build_mjcf', render):
        sim = VectorSim(values=dict(values, **replay.OPEN_TABLE),head_deg=math.degrees(start['head']),
                        lift_mm=start['lift'],noise_seed=0)
    # 0.48 s is an integer multiple of both the 60 ms publication period and 5 ms control tick.
    pre=.48
    dt=sim.dt
    dur=seg.t_stop-seg.t_cmd
    for tick in range(round((pre+dur+.72)/dt)):
        if tick==round(pre/dt): sim.set_wheel_motors(seg.left,seg.right)
        if tick==round((pre+dur)/dt): sim.stop_all_motors()
        sim.step()
    out = [row(seg.label, kind, s.time_s-pre, s.gyro_radps, s.head_angle_rad,
               s.left_wheel_speed_mmps,s.right_wheel_speed_mmps,s.lift_height_mm,
               s.x_mm,s.y_mm,s.pose_angle_rad,values['imu_pitch_offset_deg']) for s in sim.states]
    # Reproduce the real camera's sampling instead of fitting a different length of track.
    ts=[r['t_s'] for r in out]
    heads=np.interp(camera_times,ts,np.unwrap([r['heading_rad'] for r in out]))
    origin=np.column_stack([np.interp(camera_times,ts,[r[k] for r in out]) for k in ('x_mm','y_mm')])
    forward=np.column_stack([np.cos(heads),np.sin(heads)])
    lens=origin+opt.LENS_X_MM*forward
    center=opt.circle(lens)[:2]
    center_x=opt.LENS_X_MM+np.median(((center-lens)*forward).sum(axis=1))
    return out,metrics(out,inset,float(center_x))

def read_inputs(data):
    values=params.values(params.load(model='vector_2'))
    states,commands=recording.load(data/'robot.jsonl')
    spins=list(recording.segments(commands,values).values())
    assert len(spins)==4
    camera=np.genfromtxt(data/'camera-picks.csv',names=True,delimiter=',')
    centers=opt.turn_centres(data/'camera-picks.csv')['spins']
    runs=[]; truth=[]; measures=[]
    for i,seg in enumerate(spins):
        # ACK is logged after stop reaches the robot; the available issued_t marks dispatch instead.
        stop=next(c for c in commands if c.get('name')=='stop' and c['t']==seg.t_stop)
        seg=recording.Segment(seg.label,seg.kind,seg.t_cmd,stop['issued_t'],seg.left,seg.right)
        rows=[row(seg.label,'recorded',s['t']-seg.t_cmd,s['gyro'],s['head'],s['wl'],s['wr'],s['lift'],
                  *s['pose'][:2],s['yaw'],values['imu_pitch_offset_deg']) for s in states
              if seg.t_cmd-.48<=s['t']<=seg.t_stop+.72]
        truth.extend(rows); measures.append(metrics(rows,'',centers[i]['centre_x_mm']))
        times=camera[camera['turn']==i+1]['time_s']+.018-seg.t_cmd
        runs.append((seg,replay._state_at(states,seg.t_cmd),times,values))
    return runs,truth,measures

def worker(args):
    inset,runs=args
    rows=[]; scores=[]
    for run in runs:
        trace,met=simulated(inset,*run); rows.extend(trace); scores.append(met)
    return inset,rows,scores

def dump(path,rows):
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys()); w.writeheader(); w.writerows(rows)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('camera_data',type=Path); parser.add_argument('output',type=Path)
    parser.add_argument('--workers',type=int,default=4)
    args=parser.parse_args(); args.output.mkdir(exist_ok=True,parents=True)
    runs,truth,measured=read_inputs(args.camera_data)
    grid=[-1., *np.arange(0,8.01,.25)]
    results={}
    with ProcessPoolExecutor(max_workers=args.workers) as workers:
        for inset,rows,ms in workers.map(worker,((k,runs) for k in grid)):
            diffs=[m['yaw']/v['yaw']-1 for m,v in zip(ms,measured)]
            results[inset]=(rows,ms,float(np.sqrt(np.mean(np.square(diffs)))))
    best=min((k for k in grid if k>=0),key=lambda k:(results[k][2],k))
    byspin=[min((k for k in grid if k>=0),key=lambda k: abs(results[k][1][i]['yaw']/m['yaw']-1))
            for i,m in enumerate(measured)]
    selected=[-1.,0.,3.,best]
    dump(args.output/'grid.csv',[dict(inset_mm=k,rms_fractional_yaw_error=v[2]) for k,v in results.items()])
    dump(args.output/'metrics.csv',measured+[m for v in results.values() for m in v[1]])
    dump(args.output/'traces.csv',truth+[r for k in dict.fromkeys(selected) for r in results[k][0]])
    outcome=dict(inset_mm=float(best),span_mm=48-2*float(best),score=results[best][2],
                 per_spin_best_mm=[float(x) for x in byspin])
    (args.output/'selection.json').write_text(json.dumps(outcome,indent=2)+'\n')
    print(outcome)

if __name__=='__main__': main()
