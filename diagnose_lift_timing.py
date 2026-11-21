"""Compare archived lift or head movements on command and first-observed-motion clocks.

Run from the simulator Python environment. The sole calibration move is the
Nov 15 full lift raise or 32-degree head raise (--joint head). All other moving segments are evaluated after selection.
"""
import argparse
import csv
from itertools import product
from pathlib import Path
import sys

import numpy as np

from simulacrum import recording, replay


RUN_A = Path(__file__).resolve().parent / "evidence/hw-rec-20261118/run-a-rest-headlift.jsonl"


def maneuvers(values, joint="lift"):
    for label, path in (("Nov15", recording.RECORDING), ("Nov18", RUN_A)):
        states, commands = recording.load(path)
        for seg in recording.segments(commands, values).values():
            if seg.kind == joint:
                yield label, seg, states


def replay_arrays(seg, states, values):
    trace = replay.replay(seg, values, states)["trace"]
    if seg.kind == "head":
        return np.array([(t, np.degrees(stock["head"]), model["head_deg"]) for t, model, stock in trace]).T
    return np.array([(t, stock["lift"], model["lift_mm"]) for t, model, stock in trace]).T


def timing(t, height, target, values, minimum_counts=1, joint="lift"):
    baseline = np.median(height[t < 0])
    settled = np.median(height[t >= 0.82])
    if abs(settled - baseline) < 3:
        return None
    if joint == "head":
        angles = np.radians(height)
    else:
        arm = values["lift_arm_length_mm"]
        pivot = values["lift_base_position_mm"][2]
        angles = np.arcsin((height - pivot) / arm)
    counts = ((angles - np.median(angles[t < 0])) * np.sign(target - baseline)
              / values[f"{joint}_rad_per_encoder_count"])
    # Require a second moving sample. Allow 0.02 count for encoder rounding.
    moving = counts >= minimum_counts - 0.02
    i = np.flatnonzero((t[:-1] >= 0) & moving[:-1] & moving[1:])[0]
    fraction = (height - baseline) / (settled - baseline)
    landmarks = []
    for level in (0.1, 0.5, 0.9):
        j = np.flatnonzero((fraction[:-1] < level) & (fraction[1:] >= level))[0]
        landmarks.append(np.interp(level, fraction[j:j+2], t[j:j+2]))
    return np.array([t[i-1], t[i], *landmarks])


def evaluate(seg, states, values):
    t, stock, model = replay_arrays(seg, states, values)
    hw = timing(t, stock, seg.target, values, joint=seg.kind)
    if hw is None:
        return None
    sim = timing(t, model, seg.target, values, joint=seg.kind)
    return hw, sim, np.sqrt(np.mean((stock - model)**2))


def select(motion, values):
    _, seg, states = motion
    trials = []
    for accel, delay in product((6, 8, 10, 12, 15, 18), (75, 90, 105, 120, 135, 150, 165, 180)):
        candidate = dict(values, **{f"{seg.kind}_accel_default_radps2": accel, "servo_command_delay_ms": delay})
        hw, sim, error = evaluate(seg, states, candidate)
        onset_gap = abs(np.mean(hw[:2]) - np.mean(sim[:2]))
        duration_gap = abs((hw[4] - hw[2]) - (sim[4] - sim[2]))
        trials.append((accel, delay, onset_gap, duration_gap, error))
    default_delay = values["servo_command_delay_ms"]
    # Estimate acceleration by travel duration at the existing delay, then match
    # the midpoint of the onset interval. Use command-time RMS to break ties.
    accel = min((x for x in trials if x[1] == default_delay), key=lambda x: x[3])[0]
    pairs = []
    for a in (values[f"{seg.kind}_accel_default_radps2"], accel):
        pairs.append(min((x for x in trials if x[0] == a), key=lambda x: (x[2], x[4]))[:2])
    return list(dict.fromkeys(pairs))


def crossing_windows(t, height):
    start = np.median(height[t < 0])
    travel = (height - start) / (np.median(height[t >= 0.82]) - start)
    indices = [np.flatnonzero((travel[:-1] < f) & (travel[1:] >= f))[0]
               for f in np.arange(1, 10) / 10]
    return t[indices], t[np.array(indices) + 1]


def compare_sessions(motions, values):
    """Report all rigid shifts allowed by the two sampled sets of 10–90% crossings."""
    original = {}
    for run, seg, states in motions:
        t, hw, _ = replay_arrays(seg, states, values)
        if abs(np.median(hw[t >= 0.82]) - np.median(hw[t < 0])) < 50:
            continue
        direction = "raise" if hw[-1] > hw[0] else "lower"
        low, high = crossing_windows(t, hw)
        if run == "Nov15":
            original[direction] = (low, high)
        else:
            ref_low, ref_high = original[direction]
            left, right = 1000 * max(low - ref_high), 1000 * min(high - ref_low)
            print(f"{run} {seg.label} {direction}: shift must be >= {left:.0f} ms and <= {right:.0f} ms",
                  file=sys.stderr)


def compare_head_sessions(motions, values):
    # Closest same-direction excursions. The absolute start and end angles differ.
    pairs = {"head_6": "head_2", "head_1": "head_3"}
    windows = {}
    for run, seg, states in motions:
        t, hw, _ = replay_arrays(seg, states, values)
        if run == "Nov15" and seg.label in pairs.values():
            windows[seg.label] = crossing_windows(t, hw)
        elif run == "Nov18" and seg.label in pairs:
            ref_low, ref_high = windows[pairs[seg.label]]
            low, high = crossing_windows(t, hw)
            print(f"{pairs[seg.label]} -> {seg.label}: common shift interval "
                  f"{1000 * max(low - ref_high):.0f} to {1000 * min(high - ref_low):.0f} ms",
                  file=sys.stderr)


def main():
    values = replay.nominal_values()
    parser = argparse.ArgumentParser(description="Compare stock and simulated servo timing.")
    parser.add_argument("--joint", choices=("head", "lift"), default="lift")
    joint = parser.parse_args().joint
    motions = list(maneuvers(values, joint))
    if joint == "lift":
        compare_sessions(motions, values)
    else:
        compare_head_sessions(motions, values)
    fit = next(row for row in motions if (row[0], row[1].label) == ("Nov15", "head_2" if joint == "head" else "lift_1"))
    pairs = [(values[f"{joint}_accel_default_radps2"], values["servo_command_delay_ms"]), *select(fit, values)]
    writer = csv.writer(sys.stdout)
    writer.writerow(["run", "move", "accel_radps2", "delay_ms", "signal",
                     "last_rest_ms", "first_moving_ms", "ten_ms", "half_ms", "ninety_ms", "replay_rmse_deg" if joint == "head" else "replay_rmse_mm"])
    for accel, delay in pairs:
        candidate = dict(values, **{f"{joint}_accel_default_radps2": accel, "servo_command_delay_ms": delay})
        for run, seg, states in motions:
            result = evaluate(seg, states, candidate)
            if result is None:
                continue
            hw, sim, error = result
            for signal, times in (("stock", hw), ("model", sim)):
                writer.writerow([run, seg.label, accel, delay, signal, *np.rint(1000*times).astype(int),
                                 round(error, 3)])


if __name__ == "__main__":
    main()
