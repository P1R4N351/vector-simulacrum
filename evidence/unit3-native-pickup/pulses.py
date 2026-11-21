"""Summarize measured loaded and empty pulses at SDK event cadence."""

import csv, gzip, json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

here = Path(__file__).resolve().parent
fig, axes = plt.subplots(4, 4, figsize=(14, 9), sharex=True)
out = []
for archive in sorted(here.glob("series-*.jsonl.gz")):
    records = [json.loads(s) for s in gzip.open(archive, "rt")]
    states = [r for r in records if r.get("name") == "robot_state"]
    for i, r in enumerate(records):
        if r["kind"] != "pulse_begin":
            continue
        load, phase = r["action"].split()
        trial = next(e["number"] for e in reversed(records[:i]) if e["kind"] == "trial")
        end = next((e for e in records[i + 1 :] if e["kind"] == "pulse_end"), None)
        if end is None:
            continue
        sel = [s for s in states if -0.32 <= s["t"] - r["t"] < end["t"] - r["t"] + 0.39]
        t = np.array([s["t"] - r["t"] for s in sel])
        d = [s["data"] for s in sel]
        if not len(t):
            continue

        def col(key):
            return np.array([v.get(key, 0.0) for v in d])

        l, rw = col("left_wheel_speed_mmps"), col("right_wheel_speed_mmps")
        pitch = col("pose_pitch_rad") * 180 / np.pi
        lift = col("lift_height_mm")
        active = (t >= 0) & (t <= end["t"] - r["t"])
        pre = t < 0
        post = t >= 0
        if not pre.any() or not active.any():
            continue
        h = lift - np.median(lift[pre])
        p = pitch - np.median(pitch[pre])
        sign = np.sign(r["left"])
        speed = (l * np.sign(r["left"]) + rw * np.sign(r["right"])) / 2
        gyro = np.array([v["gyro"].get("z", 0) for v in d])
        miss = np.mean((abs(l[active] - r["left"]) + abs(rw[active] - r["right"])) / 2)
        out.append(
            [
                archive.stem,
                trial,
                load,
                phase,
                int(active.sum()),
                end["t"] - r["t"],
                miss,
                speed[active].max(),
                np.ptp(pitch[post]),
                max(0, -min(h[post])),
                max(abs(gyro[post])),
            ]
        )
        j = ["spin1", "spin2", "reverse", "forward"].index(phase)
        for ax, values in zip(axes[:, j], [speed, p, h, gyro]):
            ax.plot(
                t,
                values,
                ".-",
                color="#00778a" if load == "loaded" else "#c4712d",
                alpha=0.75,
                lw=1,
                ms=2,
            )
            ax.axvline(0, color=".7", lw=0.4)
        axes[0, j].set_title(phase)
for i, label in enumerate(
    [
        "Wheel speed along\ncommand (mm/s)",
        "SDK pitch change (°)",
        "Lift change (mm)",
        "Head gyro Z (rad/s)",
    ]
):
    axes[i, 0].set_ylabel(label)
for ax in axes[-1]:
    ax.set_xlabel("Seconds from command")
fig.suptitle("Stock pickup pulses · teal loaded, orange empty")
fig.tight_layout()
fig.savefig(here / "pulses.png", dpi=140)
with (here / "pulses.csv").open("w") as f:
    w = csv.writer(f, lineterminator="\n")
    w.writerow(
        [
            "series",
            "trial",
            "load",
            "phase",
            "command_samples",
            "command_s",
            "wheel_absolute_error_mmps",
            "peak_along_command_mmps",
            "pitch_range_deg",
            "max_lift_drop_mm",
            "peak_abs_gyro_z_radps",
        ]
    )
    w.writerows(out)
