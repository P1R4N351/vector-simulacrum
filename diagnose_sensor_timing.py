"""Test sensor timing against recorded stops without fitting a delay to the repeats."""
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from diagnose_braking import DATA, ROOT, BASE_LAG, segments
from replay_measured_turn import run

TICK = .005
COUNT_MM = .96 * 29 * .25 * np.pi / 172.3
CUTOFF_HZ = 74.6 / 4


def encoder_readback(t, speed):
    """Forward estimate the unsigned pulse train on one shaft within one turn."""
    grid = np.arange(t[0], t[-1] + .0001, .0001)
    velocity = np.interp(grid, t, speed)
    travel = np.r_[0, np.cumsum((abs(velocity[1:]) + abs(velocity[:-1])) * .00005)]
    predictions = []
    for fraction in (.25, .75):
        targets = np.arange(fraction * COUNT_MM, travel[-1], COUNT_MM)
        edges = np.interp(targets, travel, grid)
        for phase in (0, TICK / 2):
            ticks = np.arange(t[0] + phase, grid[-1] + TICK, TICK)
            counts = np.searchsorted(edges, ticks, side="right")
            values = np.zeros(len(ticks))
            empty = 0
            previous_edge = t[0]
            for j in range(1, len(ticks)):
                count = counts[j] - counts[j-1]
                if count:
                    edge = edges[counts[j]-1]
                    direction = np.sign(np.interp(edge, t, speed))
                    values[j] = np.trunc(direction * count * COUNT_MM / (edge - previous_edge))
                    empty = 0
                    previous_edge = edge
                else:
                    empty += 1
                    if empty < 5:
                        values[j] = values[j-1]
            indices = np.maximum(0, np.searchsorted(ticks, t, side="right") - 1)
            predictions.append(values[indices])
    return np.mean(predictions, axis=0)


def gyro_observation(clock, gyro, t, poles=1):
    """One-pole cutoff surrogate, followed by independent sample and poll phases."""
    # Equal cascaded poles, matched to the same total -3 dB bandwidth.
    pole_hz = CUTOFF_HZ / np.sqrt(2**(1/poles)-1)
    memory = np.exp(-2 * np.pi * pole_hz * (clock[1] - clock[0]))
    filtered = gyro.copy()
    for _ in range(poles):
        source = filtered.copy()
        for i in range(1, len(clock)):
            filtered[i] = memory * filtered[i-1] + (1-memory)*source[i]
    samples = []
    phases = (np.arange(4) + .5) * TICK / 4
    for poll_phase in phases:
        poll = np.floor((t-poll_phase)/TICK)*TICK + poll_phase
        for sensor_phase in phases:
            acquisition = np.floor((poll-sensor_phase)/TICK + 1e-10)*TICK + sensor_phase
            samples.append(np.interp(acquisition, clock, filtered))
    return np.mean(samples, axis=0), np.min(samples, axis=0), np.max(samples, axis=0)


def main(original, repeats):
    cases = [next(segments(original)), *segments(repeats)]
    fig, axes = plt.subplots(2, 4, figsize=(14, 6.3), sharex=True, sharey="row")
    writer_rows = []
    panel = 0
    for label, rows, t, measured, wheels, start, gap in cases:
        if gap > .18:
            print(f"{label} excluded: {gap:.3f} s gap")
            continue
        clock, base = run(rows, t, wheels, 42, dense=True)
        baseline = np.interp(t-BASE_LAG, clock, base[:, 0])
        gyro_only = gyro_observation(clock, base[:, 0], t)[0]
        # One inverse iteration about the measured profile. Zero-reading and
        # sub-frame trajectories remain ambiguous; this is an input sensitivity test.
        reported = np.column_stack([encoder_readback(t, wheel) for wheel in wheels.T])
        corrected = wheels + (wheels - reported)
        clock, adjusted = run(rows, t, corrected, 42, dense=True)
        predicted, low, high = gyro_observation(clock, adjusted[:, 0], t)
        two_pole = gyro_observation(clock, adjusted[:, 0], t, poles=2)[0]
        brake = (t >= 0) & (t <= .45)
        steady_start = t[np.flatnonzero((t >= start) & (np.min(abs(wheels),axis=1)>=90))[0]]
        phase = np.select([brake, (t>=steady_start)&(t<0)], ["brake","steady"], default="other")
        for row in zip(t, phase, measured, baseline, gyro_only, predicted, low, high, two_pole, *wheels.T, *corrected.T):
            writer_rows.append([label,*row])
        direction = 1 if "ccw" in label else -1
        view = (t>=-.15)&(t<=.42)
        ax, error = axes[:, panel]
        ax.set_title("Original CCW" if panel==0 else label.replace("_"," ").upper())
        for y, fmt, color, name in [(measured,"o-","#222936","Recorded"),(baseline,"--","#bc623e","17 ms baseline"),
                                     (predicted,"-","#147c78","Sensor model")]:
            ax.plot(t[view], direction*y[view], fmt, color=color, label=name, ms=4)
        ax.fill_between(t[view], direction*low[view], direction*high[view], color="#147c78", alpha=.20)
        for y, style, color in [(baseline,"--","#bc623e"),(predicted,"o-","#147c78")]:
            error.plot(t[view],direction*(y-measured)[view],style,color=color,ms=3)
        for axis in (ax,error):
            axis.axvline(0,color=".55",lw=.8)
            axis.grid(alpha=.18)
        error.axhline(0,color=".5",lw=.7)
        panel+=1
    axes[0,0].set_ylabel("Turn-aligned head gyro (°/s)")
    axes[1,0].set_ylabel("Prediction − recording (°/s)\nturn-aligned")
    fig.suptitle("Sensor timing through a 100 mm/s turn stop",weight="bold",fontsize=16)
    fig.legend(*axes[0,0].get_legend_handles_labels(),loc="upper center",ncol=3,bbox_to_anchor=(.5,.94))
    fig.supxlabel("Seconds from stop-command issue")
    fig.subplots_adjust(top=.81,bottom=.12,wspace=.12)
    fig.savefig(ROOT/"physics-plots"/"sensor-timing-braking.png",dpi=150)
    with (DATA/"sensor-timing-braking.csv").open("w",newline="") as stream:
        writer=csv.writer(stream)
        writer.writerow(["turn","time_s","phase","gyro_dps","baseline_dps","gyro_only_dps","sensor_dps",
                         "phase_min_dps","phase_max_dps","two_pole_dps","left_mmps","right_mmps","estimated_left_mmps","estimated_right_mmps"])
        writer.writerows(writer_rows)


if __name__ == "__main__":
    main(*(Path(arg) for arg in sys.argv[1:]))
