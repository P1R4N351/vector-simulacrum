"""Generate the board probe or compare five raw Vector captures. Requires NumPy and SciPy."""
import argparse
import csv
from pathlib import Path

import numpy as np
from scipy.io import wavfile
from scipy.signal import correlate, find_peaks, hilbert

RATE = 16000
COUNT = 8
PERIOD = 0.20
NAMES = ("open-before", "40", "60", "80", "open-after")


def pulse(rate):
    t = np.arange(round(.004 * rate)) / rate
    return np.hanning(len(t)) * np.sin(2 * np.pi * (2000*t + 500000*t*t))


def prepare(path):
    audio = np.zeros(RATE * 2)
    wave = pulse(RATE)
    for i in range(COUNT):
        start = round((.25 + PERIOD*i) * RATE)
        audio[start:start+len(wave)] = .25 * wave
    wavfile.write(path, RATE, np.round(audio * 32767).astype(np.int16))


def capture(path):
    header, pcm = wavfile.read(path)
    if header not in (15625, 16000) or pcm.ndim != 2 or pcm.shape[1] != 4 or pcm.dtype != np.int16:
        raise ValueError(f"{path}: expected four-channel signed 16-bit raw PCM at 15625 or 16000 Hz")
    if len(pcm) < 1.5 * header or np.any(np.abs(pcm.astype(float)) >= 32767):
        raise ValueError(f"{path}: short or clipped capture")
    if any(np.array_equal(pcm[:, i], pcm[:, j]) for i in range(4) for j in range(i)):
        raise ValueError(f"{path}: duplicated channels")
    profiles, rates = [], []
    for ch in range(4):
        x = pcm[:, ch].astype(float)
        corr = correlate(x, pulse(header), mode="valid")
        env = abs(hilbert(corr))
        noise = np.median(env)
        peaks, _ = find_peaks(env, height=max(.20 * max(env), 8 * noise), distance=.13 * header)
        if len(peaks) != COUNT:
            raise ValueError(f"{path} channel {ch}: found {len(peaks)} of {COUNT} direct pulses")
        rate = np.median(np.diff(peaks)) / PERIOD
        if not 15200 <= rate <= 16400 or max(abs(np.diff(peaks) - rate * PERIOD)) > 4:
            raise ValueError(f"{path} channel {ch}: unstable pulse cadence")
        times = np.arange(0, 8, .025)
        shots = []
        for p in peaks:
            end = p + round(.009 * rate)
            if p < .05 * rate or end >= len(env):
                raise ValueError(f"{path}: missing baseline or echo tail")
            shots.append(np.interp(times, (np.arange(p, end)-p)*1000/rate, env[p:end]/env[p]))
        profiles.append(np.median(shots, axis=0))
        rates.append(rate)
    return times, np.array(profiles), rates


def analyze(directory):
    loaded = [capture(directory / (name + ".wav")) for name in NAMES]
    t = loaded[0][0]
    curves = np.stack([row[1] for row in loaded])
    control = (curves[0] + curves[4]) / 2
    gate = (t >= 1.4) & (t <= 6.5)
    drift = np.max(abs(curves[4] - curves[0])[:, gate], axis=1)
    with (directory / "profiles.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["condition", "channel", "delay_ms", "normalized_envelope"])
        for name, (_, profiles, rates) in zip(NAMES, loaded):
            print(name, "cadence samples/s", [round(r) for r in rates])
            for ch, profile in enumerate(profiles):
                w.writerows((name, ch, round(delay, 3), value) for delay, value in zip(t, profile))
    lags, strong = [], []
    for name, curve in zip(NAMES[1:4], curves[1:4]):
        excess = (curve - control)[:, gate]
        idx = np.argmax(excess, axis=1)
        lag = t[gate][idx]
        lags.append(lag)
        strong.append(excess[np.arange(4), idx] > np.maximum(.015, 3 * drift))
        print(name, "cm excess peak delays (ms)", lag.round(3).tolist())
    steps = np.diff(lags, axis=0)
    ok = np.all(strong, axis=0) & np.all(abs(steps - 1.166) < .35, axis=0)
    print(f"{sum(ok)}/4 channels pass both board shifts and open controls")
    return sum(ok) >= 3


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["prepare", "analyze"])
    parser.add_argument("path", type=Path, help="output probe WAV or directory of captures")
    args = parser.parse_args()
    if args.action == "prepare":
        prepare(args.path)
    else:
        raise SystemExit(0 if analyze(args.path) else 1)
