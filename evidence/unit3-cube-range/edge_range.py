"""Compare camera-edge and odometry pose inputs to the same browser range sensor."""
import csv
import json
from pathlib import Path
import subprocess
import sys

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
root = here.parents[1]
sys.path.insert(0, str(here.parent/'unit3-cube-contact-replay'))
from replay import K, rotation, start_pose

def table(path):
    return np.genfromtxt(path, names=True, delimiter=',')

cal = json.loads((root/'web/camera_calibration_vector2.json').read_text())
K[:2] = [[cal['fx_px'], 0, cal['cx_px']], [0, cal['fy_px'], cal['cy_px']]]
edges = table(here.parent/'unit3-cube-camera-scale/edges.csv')
states = table(here.parent/'unit3-cube-contact-replay/inputs.csv')
headings = table(here.parent/'unit3-printed-fork-contact/overhead-yaw.csv')
picks = json.loads((here.parent/'unit3-paired-cube-push/endpoint-picks.json').read_text())

def from_edge(row, head, yaw):
    # Transform pinhole rays to chassis forward, left, up; no lens distortion supplied.
    pitch = head - np.deg2rad(4)
    c, s = np.cos(pitch), np.sin(pitch)
    camera_to_body = np.array([[0,s,c],[-1,0,0],[0,-c,s]])
    uv = [[row['u0'], row['v0'], 1], [row['u1'], row['v1'], 1]]
    rays = camera_to_body @ np.linalg.solve(K, np.array(uv).T)
    right = np.array([np.sin(yaw), -np.cos(yaw), 0])
    matrix = np.column_stack((-rays[:,0], rays[:,1]))
    depth = np.linalg.lstsq(matrix, 23*right, rcond=None)[0]
    assert min(depth) > 0
    midpoint = (rays @ depth)[:2]/2
    camera_x = -13 + 18.84*np.cos(head) + 7.96*np.sin(head)
    return midpoint + [camera_x,0] + 22*np.array([np.cos(yaw), np.sin(yaw)])

fig, axes = plt.subplots(1,5,figsize=(13,3.5),sharey=True,layout='constrained')
driver = ['node',str(root/'web/tools/cube-range-replay.mjs')]
with (here/'edge-range.csv').open('w') as out:
    writer = csv.writer(out,lineterminator='\n')
    writer.writerow(['trial','receipt_s','aligned_s','physical_mm','odom_mm','edge_mm','anchored_edge_mm'])
    for trial, ax in enumerate(axes,1):
        p = next(p for p in picks if p['trial']==trial and p['camera']=='on')
        origin = rotation(np.deg2rad(p['state']['yaw'])) @ start_pose(p)[0] + p['state']['xy']
        yaw = np.deg2rad(headings[headings['trial']==trial]['degrees'][0])
        path = states[states['trial']==trial]
        e = edges[(edges['trial']==trial)&(edges['seconds']<2.75)]
        # Capture clock and lag frozen by capture_clock.py on trials 1 and 2.
        cap = e['capture_ms']*.001
        t = cap + np.median(e['seconds']-cap) - .236058754 + 1.5
        x,y,h,head,lift = [np.interp(t,path['sec'],path[k])
                            for k in ('x_mm','y_mm','yaw_deg','head_rad','lift_mm')]
        h = np.deg2rad(h)
        odom = np.array([rotation(-angle)@(origin-[cx,cy]) for cx,cy,angle in zip(x,y,h)])
        optic = np.array([from_edge(row,ha,yaw-angle) for row,ha,angle in zip(e,head,h)])
        rest = e['seconds'] < .5
        anchor = np.median(odom[rest]-optic[rest],axis=0)
        runs = [dict(trial=trial,poses=np.column_stack([xy,yaw-h,lift]).tolist())
                for xy in (odom,optic,optic+anchor)]
        results = np.array(json.loads(subprocess.check_output(driver,input=json.dumps(runs),text=True)))
        assert np.all(results[:,:,1])
        # Compare at the previously measured relative range lag; never extrapolate moving samples.
        time = t + .071
        moving = e['seconds'] > 1
        assert np.all((time[moving]>=path['sec'][0])&(time[moving]<=path['sec'][-1]))
        assert np.all(path['found'][(path['sec']>=0)&(path['sec']<=4.25)])
        physical = np.interp(time,path['sec'],path['prox_mm'])
        for i in np.flatnonzero(moving):
            writer.writerow([trial,e['seconds'][i],t[i]-1.5,physical[i],*results[:,i,0]])
        ax.plot(t[moving]-1.5,physical[moving],'ko-',label='Recorded')
        for values,style,label in zip(results,['s--','^--','x:'],['Odometry','Upper edge','Upper edge, anchored']):
            ax.plot(t[moving]-1.5,values[moving,0],style,label=label)
        ax.set(title=f'Trial {trial}',xlabel='Time from drive (s)')
axes[0].set_ylabel('Range (mm)')
axes[-1].legend(fontsize=7)
fig.savefig(here/'edge-range.png',dpi=150)
