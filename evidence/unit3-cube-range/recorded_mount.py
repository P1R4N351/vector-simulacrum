"""Replay wheel targets in the shelf coordinate frame estimated from the last dock observation."""
import csv
import io
import json
import subprocess

import matplotlib.pyplot as plt
import numpy as np

from return_braking import ROOT, states, table


def read_trial(contact, observation):
    assert contact['session'] == observation['session']
    rows, _ = states(contact['session'])
    t = np.array([float(r['receipt_s']) for r in rows]) - contact['first_on_s']
    wheels = np.array([[float(r['left_mmps']), float(r['right_mmps'])] for r in rows])
    # First three consecutive backing samples after the final turn's last opposite-sign fast wheels.
    turning = (np.min(wheels, axis=1) < -15) & (np.max(wheels, axis=1) > 15)
    turn_end = np.flatnonzero(turning & (t < 0) & (t > -8))[-1]
    k = next(i for i in range(turn_end+1, len(t)-2) if np.all(wheels[i:i+3] < -12))
    start = np.flatnonzero((np.arange(len(t)) < k) & (np.arange(len(t)) >= turn_end)
                           & (np.abs(wheels).max(axis=1) < 3))[-1]
    sel = (t >= t[start]) & (t <= .5)
    t, wheels = t[sel], wheels[sel]
    a, b, c, d = [observation['q'+str(i)] for i in range(4)]
    outward = -np.array([1-2*(c*c+d*d), 2*(a*d+b*c)])
    outward /= np.linalg.norm(outward)
    xy = np.array([[float(r['x_mm']),float(r['y_mm'])] for r in rows])[sel]
    xy -= [observation['charger_x'], observation['charger_y']]
    out = xy @ outward
    lat = xy @ [-outward[1], outward[0]]
    yaw = np.array([float(r['yaw_rad']) for r in rows])[sel] - np.arctan2(*outward[::-1])
    yaw = np.rad2deg(np.arctan2(np.sin(yaw), np.cos(yaw)))
    trial = dict(name=contact['session'], out=out[0], lat=lat[0], yaw=yaw[0],
                 head=np.rad2deg(float(rows[start]['head_rad'])), lift=float(rows[start]['lift_mm']),
                 wheels=np.column_stack([t, wheels]).tolist())
    return trial, (t, out, lat, yaw, wheels)


def main():
    trials = [read_trial(c, o) for c, o in zip(table('return-scores.csv'), table('mount-target-inputs.csv'), strict=True)]
    stdout = subprocess.check_output(['node', str(ROOT/'recorded_mount.mjs')],
                                     input=json.dumps([trial for trial,_ in trials]), text=True)
    sim = np.genfromtxt(io.StringIO(stdout), delimiter=',', names=True, dtype=None, encoding='utf8')
    fig, axs = plt.subplots(3, 5, figsize=(14, 7), sharex='col', sharey='row', layout='constrained')
    scores = []
    for axes, (trial, real) in zip(axs.T, trials):
        t, out, lat, yaw, wheels = real
        before = t < 0
        for ax, signal in zip(axes, [out, lat, wheels.mean(axis=1)]):
            ax.plot(t[before],signal[before], color='black', lw=1.2, label='Recorded SDK')
            ax.axvline(0, color='gray', lw=.7)
        axes[0].set_title(trial['name'])
        axes[-1].set_xlabel('Time from recorded contact (s)')
        for variant, label in [('canonical', 'Canonical'),('driven_rear_envelope','Driven rear support'),
                               ('driven_mean_only','Driven, equal targets')]:
            run = sim[(sim['session']==trial['name'])&(sim['variant']==variant)]
            contact = run['t_s'][run['on']==1]
            average = (run['left_mmps']+run['right_mmps'])/2
            for ax, signal in zip(axes,[run['out_mm'],run['lat_mm'],average]):
                ax.plot(run['t_s'],signal,lw=.9,label=label)
            errors = np.interp(t[before],run['t_s'],average)-wheels[before].mean(axis=1)
            scores.append([trial['name'],variant,t[0],out[0],lat[0],yaw[0],out[before][-1],lat[before][-1],
                           np.interp(0,run['t_s'],run['out_mm']),np.interp(0,run['t_s'],run['lat_mm']),
                           np.mean(np.abs(errors)),contact[0] if len(contact) else ''])
    for ax, label in zip(axs[:,0],['Outward from lip (mm)','Lateral (mm)','Mean wheel speed (mm/s)']):
        ax.set_ylabel(label)
    axs[0,-1].legend(fontsize=7)
    fig.savefig(ROOT/'recorded-mount.png',dpi=160)
    with (ROOT/'recorded-mount.csv').open('w') as f:
        w=csv.writer(f,lineterminator='\n')
        w.writerow('session variant start_s start_out_mm start_lat_mm start_yaw_deg sdk_last_off_out_mm sdk_last_off_lat_mm sim_at_on_out_mm sim_at_on_lat_mm wheel_mae_mmps first_contact_s'.split())
        w.writerows(scores)


if __name__=='__main__':
    main()
