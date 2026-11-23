"""Compare accumulated yaw before the SDK relocalizes to the charger."""
import csv
import io
import json
import subprocess

import matplotlib.pyplot as plt
import numpy as np

from recorded_mount import ROOT, read_trial, states, table


def main():
    trials = [read_trial(c, o) for c, o in zip(table("return-scores.csv"), table("mount-target-inputs.csv"), strict=True)]
    output = subprocess.check_output(["node", str(ROOT / "recorded_mount.mjs")],
                                     input=json.dumps([trial for trial, _ in trials]), text=True)
    sim = np.genfromtxt(io.StringIO(output), names=True, delimiter=",", dtype=None, encoding="utf8")
    figure, axes = plt.subplots(1, 5, figsize=(14, 3.8), sharey=True, layout="constrained")
    scores = []
    for ax, (trial, real), contact in zip(axes, trials, table("return-scores.csv"), strict=True):
        name = trial["name"]
        t, _, _, heading, wheels = real
        before = t < 0
        t = t[before]
        sdk = np.rad2deg(np.unwrap(np.deg2rad(heading[before]))) - heading[0]
        rows, _ = states(name)
        rows = [r for r in rows if t[0] <= float(r["receipt_s"]) - contact["first_on_s"] < 0]
        assert len(rows) == len(t)
        rate = np.array([float(r["gz"]) / np.cos(float(r["head_rad"])) for r in rows])
        gyro = np.rad2deg(np.r_[0, np.cumsum((rate[1:] + rate[:-1]) * .5 * np.diff(t))])
        projected = np.array([float(r["gx"]) * np.sin(float(r["head_rad"])) +
                              float(r["gz"]) * np.cos(float(r["head_rad"])) for r in rows])
        even_clock = np.linspace(t[0], t[-1], len(t))
        uniform = np.rad2deg(np.r_[0, np.cumsum((projected[1:] + projected[:-1]) * .5 * np.diff(even_clock))])
        wheel_rate = (wheels[before, 1] - wheels[before, 0]) / 42
        wheel_angle = np.rad2deg(np.r_[0, np.cumsum((wheel_rate[1:] + wheel_rate[:-1]) * .5 * np.diff(t))])
        ax.plot(t, wheel_angle, ":", color="purple", lw=1.1, label="Wheel difference / 42 mm")
        ax.plot(even_clock, uniform, "--", color="gray", lw=1, label="Two-axis, even cadence")
        ax.plot(t, sdk, label="SDK pose")
        ax.plot(t, gyro, label="Head gyro integral")
        for variant, label in [("canonical", "Canonical"), ("driven_rear_envelope", "Driven rear support")]:
            run = sim[(sim["session"] == name) & (sim["variant"] == variant)]
            yaw = np.rad2deg(np.unwrap(np.deg2rad(run["yaw_deg"]))) - run["yaw_deg"][0]
            ax.plot(t, np.interp(t, run["t_s"], yaw), label=label)
            if variant == "driven_rear_envelope":
                station = run["t_s"][run["out_mm"] <= 35][0]
                ax.axvline(station, color="gray", ls=":", lw=1)
                for phase, end in [("35mm", station), ("last_off", t[-1])]:
                    scores.append([name, phase, end, np.interp(end, t, sdk), np.interp(end, t, gyro),
                                   np.interp(end, run["t_s"], yaw), np.interp(end, even_clock, uniform),
                                   np.interp(end, t, wheel_angle)])
        ax.set_title(f"Return {name}")
        ax.set_xlabel("Time from first on-report (s)")
    axes[0].set_ylabel("Yaw change from backup start (°)")
    axes[0].legend(fontsize=7)
    figure.suptitle("Reverse docking before charger acquisition · dotted line marks simulated 35 mm station")
    figure.savefig(ROOT / "mount-yaw.png", dpi=160)
    with (ROOT / "mount-yaw.csv").open("w") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["session","endpoint","t_s","sdk_deg","gyro_deg","driven_model_deg","gyro_two_axis_even_deg","wheel_42mm_deg"])
        writer.writerows(scores)


if __name__ == "__main__":
    main()
