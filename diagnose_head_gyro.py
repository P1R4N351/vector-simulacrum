"""Offline pitch-gyro onset and speed-angle replay; writes CSV to stdout and scores to stderr."""
import csv
import math
import sys
from unittest.mock import patch
from xml.etree import ElementTree as ET

import numpy as np

from diagnose_lift_timing import maneuvers
from simulacrum import mjcf, replay


class GyroReplay(replay.VectorSim):
    def reported_now(self):
        signals = super().reported_now()
        signals["pitch_speed"] = -math.degrees(self._imu()[1][1])
        return signals


def shifted_head(xml, x_mm, z_mm):
    """Move the diagnostic box's mass centre only, retaining mass, shape and central inertia."""
    root = ET.fromstring(xml)
    head = root.find(".//body[@name='head']")
    box = head.find("geom")
    x, y, z = np.fromstring(box.get("size"), sep=" ") ** 2
    mass = float(box.get("mass"))
    ET.SubElement(head, "inertial", pos=mjcf.vec([x_mm*.001, 0, z_mm*.001]), mass=str(mass),
                  diaginertia=mjcf.vec(mass/3 * np.array([y+z, x+z, x+y])))
    return ET.tostring(root, encoding="unicode")


def signals(motion, values, delay, x, z):
    _, seg, states = motion
    build = mjcf.build_mjcf
    def model(v, obstacles=()):
        xml = build(v, obstacles)
        return shifted_head(xml, x, z) if x or z else xml
    with patch.object(replay, "VectorSim", GyroReplay), patch.object(mjcf, "build_mjcf", model):
        tr = replay.replay(seg, dict(values, servo_command_delay_ms=delay), states)["trace"]
    return np.array([[t, math.degrees(h["head"]), -math.degrees(h["gyro"][1]),
                      m["head_deg"], m["pitch_speed"]] for t, m, h in tr]).T


def onset(t, speed, direction, threshold=1.0):
    """Bracket first sustained directional speed, interpolate threshold and estimate a ramp intercept."""
    v = direction * (speed - np.median(speed[t < 0]))
    over = v >= threshold
    hits = np.flatnonzero((t[:-1] >= 0) & over[:-1] & over[1:])
    if not len(hits):
        return np.full(4, np.nan)
    i = hits[0]
    crossing = np.interp(threshold, v[i-1:i+1], t[i-1:i+1])
    # First two moving samples give a local constant-acceleration extrapolation.
    intercept = np.nan
    if v[i+1] > v[i]:
        est = t[i] - v[i] * (t[i+1]-t[i])/(v[i+1]-v[i])
        if t[i-1] <= est <= t[i]:
            intercept = est
    return np.array([t[i-1], t[i], crossing, intercept])


def rms(diff):
    return np.sqrt(np.mean(diff**2))


def increment_error(t, angle, speed, lag):
    """Compare encoder increments with the integral of linearly interpolated gyro samples."""
    v = speed - np.median(speed[t < 0])
    u = t + lag
    valid = (t[:-1] >= 0) & (t[1:] <= .82) & (u[:-1] >= t[0]) & (u[1:] <= t[-1])
    index = np.clip(np.searchsorted(t, u, side="right") - 1, 0, len(t)-2)
    integral = np.r_[0, np.cumsum(np.diff(t)*(v[:-1]+v[1:])/2)]
    dt = u - t[index]
    at = integral[index] + v[index]*dt + .5*dt**2*np.diff(v)[index]/np.diff(t)[index]
    return rms((np.diff(angle)-np.diff(at))[valid])


def main():
    values = replay.nominal_values()
    moves = list(maneuvers(values, "head"))
    fit = next(m for m in moves if m[0] == "Nov15" and m[1].label == "head_2")
    def score(candidate):
        delay, x, z = candidate
        t, h, g, model_h, model_g = signals(fit, values, delay, x, z)
        a = onset(t, g, 1)[2]
        b = onset(t, model_g, 1)[2]
        return abs(a-b), rms(model_g-g)
    base = (values["servo_command_delay_ms"], 0, 0)
    ft, fh, fg, _, _ = signals(fit, values, *base)
    lag = min(np.arange(-60, 61, 5)*.001, key=lambda d: increment_error(ft, fh, fg, d))
    print("Diagnostic gyro alignment (s):", lag, file=sys.stderr)
    latency = min([(d, 0, 0) for d in range(75,216,5)], key=score)
    # Compare possible offsets at the same candidate delay, without changing collision geometry.
    offsets = [(0,0), (4,0), (-4,0), (0,4), (0,-4)]
    posture_only = min([(base[0], x, z) for x,z in offsets], key=lambda c: score(c)[1])
    posture = min([(latency[0], x, z) for x,z in offsets], key=lambda c: score(c)[1])
    writer = csv.writer(sys.stdout)
    writer.writerow(["trial","session","move","seconds","stock_angle_deg","stock_speed_dps","sim_angle_deg","sim_speed_dps"])
    for name, candidate in [("baseline",base), ("gyro-delay",latency), ("com-only",posture_only), ("head-com",posture)]:
        print(name, candidate, file=sys.stderr)
        for motion in moves:
            run, seg, _ = motion
            t,h,g,m,s = signals(motion, values, *candidate)
            if name == "baseline":
                print(run, seg.label, "encoder increment RMS, direct / aligned (deg)",
                      increment_error(t,h,g,0), increment_error(t,h,g,lag), file=sys.stderr)
                direction = np.sign(np.median(h[t >= .82]) - np.median(h[t < 0]))
                count = np.degrees(values["head_rad_per_encoder_count"])
                ahead = direction * (h - np.median(h[t < 0])) >= .98 * count
                hits = np.flatnonzero((t[:-1] >= 0) & ahead[:-1] & ahead[1:])
                enc = 1000*t[hits[0]] if len(hits) else np.nan
                print(run, seg.label, "gyro bracket / threshold / ramp intercept ms",
                      np.round(1000 * onset(t, g, direction), 1), "encoder first ms", enc, file=sys.stderr)
            print(run,seg.label,"angle/speed RMSE",round(rms(h-m),3),round(rms(g-s),3),file=sys.stderr)
            writer.writerows((name,run,seg.label,*row) for row in zip(t,h,g,m,s))


if __name__ == "__main__":
    main()
