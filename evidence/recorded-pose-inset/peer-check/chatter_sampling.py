"""Separate gyro bandwidth from publication cadence for the frozen 3.75 mm proxy geometry."""
import argparse
import csv
import json
import math
from pathlib import Path
from unittest.mock import patch

import mujoco
import numpy as np
from scipy.signal import lfilter, lfilter_zi
from contact import contact_xml
from simulacrum import mjcf, params, recording, replay
from simulacrum.robot import VectorSim


def write(path, rows):
    with path.open("w", newline="") as handle:
        w = csv.DictWriter(handle, fieldnames=rows[0])
        w.writeheader()
        w.writerows(rows)


def rotate(g, theta):
    theta = np.asarray(theta)
    return np.column_stack((g[:, 0]*np.sin(theta)+g[:, 2]*np.cos(theta),
                            g[:, 0]*np.cos(theta)-g[:, 2]*np.sin(theta),g[:, 1]))


def run_sim(v,s,stage,pre):
    original = mjcf.build_mjcf
    with patch.object(mjcf, "build_mjcf", lambda *a:contact_xml(original(*a),3.75)):
        sim = VectorSim(values=dict(v,**replay.OPEN_TABLE),head_deg=math.degrees(s["head"]),
                        lift_mm=s["lift"],noise_seed=0)
    rows = []
    step = mujoco.mj_step
    def traced(model,data,nstep=1):
        for _ in range(nstep):
            step(model,data)
            rows.append([data.time-pre,sim.head_angle_rad(),*data.sensordata[sim._gyro_adr:sim._gyro_adr+3]])
    sim_time0=sim.data.time
    with patch.object(mujoco,"mj_step",traced):
        for tick in range(round((pre+stage.t_stop-stage.t_cmd)/sim.dt)):
            if tick == round(pre/sim.dt):
                sim.set_wheel_motors(stage.left,stage.right)
            sim.step()
    data = np.array(rows)
    data[:,0] -= sim_time0
    return data


def gyro_filtered(values,dt,poles):
    fc = 18.65 / np.sqrt(2**(1/poles)-1)
    decay = np.exp(-2*np.pi*fc*dt)
    out = values
    for _ in range(poles):
        out,_ = lfilter([1-decay], [1,-decay], out, axis=0,
                        zi=lfilter_zi([1-decay],[1,-decay])[:,None]*out[:1])
    return out


def average_windows(values,dt,window):
    n = round(window/dt)
    cs = np.cumsum(np.vstack((np.repeat(values[:1],n,axis=0),values)),axis=0)
    return (cs[n:]-cs[:-n])/n


def score(spin,unit,source,phase,t,g,theta,window):
    ok=(t>=window[0]-1e-8)&(t<=window[1]+1e-8)
    rates=np.mean(abs(rotate(g,theta)[ok]),axis=0)
    return dict(unit=unit,spin=spin,source=source,phase_ms=phase,n=int(sum(ok)),
                yaw=rates[0],roll=rates[1],pitch=rates[2])


def spin_scores(label,unit,rows,raw,window,offset):
    tt,gg,hh=zip(*rows)
    out=[score(label,unit,"recorded",0,np.array(tt),np.array(gg),np.array(hh)+offset,window)]
    t,theta,g=raw[:,0],raw[:,1]+offset,raw[:,2:]
    dt=np.median(np.diff(t))
    variants = {"instant":g,"onepole_18.65Hz":gyro_filtered(g,dt,1),
                "twopole_18.65Hz":gyro_filtered(g,dt,2),
                "boxcar_5ms":average_windows(g,dt,.005),
                "boxcar_60ms_hypothetical":average_windows(g,dt,.060)}
    for name,filtered in variants.items():
        out.append(score(label,unit,name+"_dense",-1,t,filtered,theta,window))
        for offset_ms in range(0,60,5):
            # _tick_once samples the IMU before advancing the 5 ms tick during which the state publishes.
            output_times=np.arange(offset_ms/1000, t[-1]+.01,.06)
            ix=np.maximum(0,np.searchsorted(t,output_times-.005+1e-8,side="right")-1)
            out.append(score(label,unit,name,offset_ms,output_times,filtered[ix],theta[ix],window))
    return out


def native_summary(native,sdk):
    raw=list(csv.DictReader(native.open()))
    sdk_rows=[json.loads(line) for line in sdk.open() if line.strip().endswith("}")]
    sdk_rows=[r for r in sdk_rows if r.get("k")=="s"]
    result=[]; matched=[]
    for spin in dict.fromkeys(r['turn'] for r in raw):
        rs=[r for r in raw if r['turn']==spin]
        for r in rs:
            if r['sdk_match']!='1':
                continue
            match=[q for q in sdk_rows if q['wl']==float(r['left_mmps']) and q['wr']==float(r['right_mmps'])
                   and abs(q['gyro'][2]-float(r['head_gz']))<1e-8]
            if len(match)!=1:
                continue
            q=match[0]
            err=np.array(q['gyro'])-np.array([float(r['head_'+k]) for k in ('gx','gy','gz')])
            matched.append(dict(spin=spin,t_ms=int(r['ms_from_last_edge']), max_abs_error=float(max(abs(err)))))
        for source,sub in [('native_all',rs),('sdk_matched',[r for r in rs if r['sdk_match']=='1'])]:
            t=np.array([float(r['ms_from_last_edge']) for r in sub])*.001
            g=np.array([[float(r['head_'+k]) for k in ('gx','gy','gz')] for r in sub])
            theta=np.array([float(r['head_rad']) for r in sub])+np.radians(-.05)
            result.append(score(spin,"unit 3_100mmps",source,0,t,g,theta,(-.5,-.2)))
    return result,matched


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('p2',type=Path); p.add_argument('c7',type=Path); p.add_argument('native',type=Path)
    p.add_argument('simultaneous_sdk',type=Path);p.add_argument('out',type=Path)
    a=p.parse_args(); a.out.mkdir(exist_ok=True,parents=True)
    summaries=[]
    for unit,model,path,pre in [('unit 3','vector_2',a.p2,.48),('unit 1','vector_1',a.c7,.3)]:
        v=params.values(params.load(model=model)); states,commands=recording.load(path)
        for seg in recording.segments(commands,v).values():
            if seg.kind!='wheels' or seg.left*seg.right>=0:continue
            if unit=='unit 3':
                stop=next(c for c in commands if c.get('name')=='stop' and c['t']==seg.t_stop)
                seg=recording.Segment(seg.label,seg.kind,seg.t_cmd,stop['issued_t'],seg.left,seg.right)
            steady=(.7,1.4) if unit=='unit 3' else ((seg.t_stop-seg.t_cmd)/2,seg.t_stop-seg.t_cmd)
            rows=[(r['t']-seg.t_cmd,r['gyro'],r['head']) for r in states if seg.t_cmd<=r['t']<=seg.t_stop]
            dense=run_sim(v,replay._state_at(states,seg.t_cmd),seg,pre)
            summaries.extend(spin_scores(seg.label,unit,rows,dense,steady,np.radians(v['imu_pitch_offset_deg'])))
    write(a.out/'sampling_scores.csv',summaries)
    native,matched=native_summary(a.native,a.simultaneous_sdk)
    write(a.out/'native_scores.csv',native);write(a.out/'sdk_native_matches.csv',matched)


if __name__=='__main__':main()
