"""Compare heat-loss perturbations and post-stop angular travel against SDK gyro."""
from dataclasses import replace
from functools import partial
from pathlib import Path
import sys
from unittest.mock import patch

import matplotlib.pyplot as plt
import numpy as np

here = Path(__file__).resolve().parent
sys.path[:0] = [str(here.parent/'recorded-pose-inset'), str(here.parents[1])]
import fit_p2
from contact import contact_xml

runs, truth, measured = fit_p2.read_inputs(here.parent/'camera-spins')
summary = []
fig, panels = plt.subplots(3, 2, figsize=(9, 8), layout='constrained')
for ((seg, start, times, values), obs), axes in zip(zip(runs[-2:], measured[-2:]), panels.T):
    stop = seg.t_stop-seg.t_cmd
    hardware = [r for r in truth if r['spin']==seg.label]
    signed = replace(seg, left=np.copysign(obs['wl'], seg.left), right=np.copysign(obs['wr'], seg.right))
    def gyro(trace, t):
        return np.interp(t, [r['t_s'] for r in trace], [r['yaw'] for r in trace])
    # Shared integration boundaries, including every native SDK sample in this interval.
    def braking(trace, anchor):
        t = sorted({anchor, anchor+.30, *(r['t_s'] for r in trace if anchor<r['t_s']<anchor+.30)})
        return np.rad2deg(np.trapezoid(gyro(trace,t),t))*np.sign(seg.right)
    def wheel_crossing(trace):
        threshold = .8*(obs['wl'] + obs['wr'])/2
        t = np.array([r['t_s'] for r in trace])
        speed = np.array([(abs(r['wl'])+abs(r['wr']))/2 for r in trace])
        i = np.flatnonzero((t[1:]>=stop)&(speed[:-1]>=threshold)&(speed[1:]<threshold))[0]
        return t[i]+(t[i+1]-t[i])*(threshold-speed[i])/(speed[i+1]-speed[i])
    axes[0].plot([r['t_s']-stop for r in hardware],[r['yaw'] for r in hardware], 'k.-',label='P2Y8')
    def aligned_plot(trace, label, style):
        axes[2].plot([r['t_s']-wheel_crossing(trace) for r in trace],[r['yaw'] for r in trace],style,label=label)
    aligned_plot(hardware,'P2Y8','k.-')
    rates = np.array(values['tread_heat_loss_rate_per_s'])
    for fraction in (0, .0001, .001, .01, .1, .5, 1):
        rates_used = rates + fraction*(rates.mean()-rates)
        log = []
        original = fit_p2.VectorSim._drive_treads
        def drive(robot, left, right):
            previous = np.array(robot.tread_heat)
            original(robot, left, right)
            loss = 100*rates_used*values['tread_heat_tau_s']*previous
            log.append((robot.data.time-.48, *loss, *[c.power for c in robot._controllers]))
        with patch.object(fit_p2.VectorSim,'_drive_treads',drive), \
             patch.object(fit_p2,'contact_xml',partial(contact_xml,forward_shift_mm=1)):
            trace, m = fit_p2.simulated(3.75,signed,start,times,dict(values,tread_contact_inset_mm=0,
                                             tread_heat_loss_rate_per_s=rates_used))
        duty = np.asarray(log)
        window = duty[(duty[:,0]>=.7)&(duty[:,0]<=1.4)]
        summary.append(dict(spin=seg.label, balance_fraction=fraction,
            peak_steady_torque_loss_pct=window[:,1:3].max(),mean_abs_duty=np.abs(window[:,3:]).mean(),
            yaw_error_pct=100*(m['yaw']/obs['yaw']-1),real_braking_deg=braking(hardware,stop),
            sim_braking_deg=braking(trace,stop),real_wheel_crossing_ms=1000*(wheel_crossing(hardware)-stop),
            sim_wheel_crossing_ms=1000*(wheel_crossing(trace)-stop),
            real_wheel_aligned_deg=braking(hardware,wheel_crossing(hardware)),
            sim_wheel_aligned_deg=braking(trace,wheel_crossing(trace))))
        if fraction in (0,1):
            aligned_plot(trace,f'{fraction:g} balanced','.-')
            axes[0].plot([r['t_s']-stop for r in trace],[r['yaw'] for r in trace],'.-',label=f'{fraction:g} balanced')
    subset = [r for r in summary if r['spin']==seg.label]
    axes[0].set(xlim=(-.35,.4),title=seg.label,xlabel='Seconds from stop dispatch',ylabel='Yaw rate (rad/s)')
    axes[2].set(xlim=(-.2,.3),xlabel='Seconds from 80% wheel-speed crossing',ylabel='Yaw rate (rad/s)')
    axes[2].axvline(0,color='gray',lw=.8)
    axes[0].axvline(0,color='gray',lw=.8)
    axes[1].plot(range(len(subset)),[r['yaw_error_pct'] for r in subset],'o-')
    axes[1].set(xticks=range(len(subset)),xticklabels=[f"{r['balance_fraction']:g}" for r in subset],
                xlabel='Fraction toward equal heat-loss rates',ylabel='Steady yaw error (%)')
panels[0,0].legend(fontsize=8)
fit_p2.dump(here/'heat-transient.csv',summary)
fig.savefig(here/'heat-transient.png',dpi=160)
