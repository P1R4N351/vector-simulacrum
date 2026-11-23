"""Extend measured backing from the physical contact time up to seven seconds total."""
import csv
import io
import json
import subprocess

import matplotlib.pyplot as plt
import numpy as np

from recorded_mount import ROOT, read_trial, table


def main():
    trials = []
    for contact, observation in zip(table("return-scores.csv"), table("mount-target-inputs.csv"), strict=True):
        trial, _ = read_trial(contact, observation)
        off = [sample for sample in trial["wheels"] if sample[0] < 0]
        for name, speeds in [("recorded", None), ("last", off[-1][1:]), ("straight", [-30, -30])]:
            samples = trial["wheels"] if speeds is None else off + [[0, *speeds]]
            trials.append(dict(trial, name=trial["name"] + name, wheels=samples, duration_s=7))
    raw = subprocess.check_output(["node", str(ROOT/"recorded_mount.mjs")], input=json.dumps(trials), text=True)
    data = np.genfromtxt(io.StringIO(raw), names=True, delimiter=",", dtype=None, encoding="utf8")
    scores = []
    fig, axes = plt.subplots(2, 5, figsize=(14, 6), sharex="col", sharey="row", layout="constrained")
    for col, name in enumerate("ABCDE"):
        for idx, variant in enumerate(["canonical", "driven_rear_envelope"]):
            ax = axes[idx, col]
            for case, label in [("recorded","Recorded tail"), ("last","Hold last pair"), ("straight","Both −30 mm/s")]:
                run = data[(data["session"] == name+case) & (data["variant"] == variant)]
                on = np.flatnonzero(run["on"])
                end = on[0] if len(on) else len(run)-1
                track = run[:end+1]
                tail = track[track["t_s"] >= 0]
                wheel_travel = inward = ""
                if len(tail) >= 2:
                    wheel_travel = -np.trapezoid((tail["left_mmps"]+tail["right_mmps"])/2, tail["t_s"])
                    inward = tail["out_mm"][0] - tail["out_mm"][-1]
                scores.append([name, variant, case, run["t_s"][end] if len(on) else "",
                               run["out_mm"][end], run["lat_mm"][end], run["yaw_deg"][end], wheel_travel, inward])
                line, = ax.plot(track["t_s"], track["out_mm"], label=label, lw=1.3)
                ax.plot(track["t_s"][-1], track["out_mm"][-1], "o" if len(on) else "x", color=line.get_color(), ms=5)
            ax.axvline(0, color="gray", lw=.8, ls=":")
            ax.set_title(f"{name} · " + ("Canonical" if idx == 0 else "Driven support"))
            if idx:
                ax.set_xlabel("From physical on-report (s)")
    for ax in axes[:,0]:
        ax.set_ylabel("Outward from dock lip (mm)")
    axes[0,0].legend(fontsize=7)
    fig.suptitle("Seven seconds of backing from recorded start · circle = first contact, × = timeout")
    fig.savefig(ROOT/"mount-duration.png", dpi=160)
    with (ROOT/"mount-duration.csv").open("w") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow("session model tail first_on_s endpoint_out_mm endpoint_lateral_mm endpoint_yaw_deg post_on_reverse_wheel_mm post_on_inward_mm".split())
        writer.writerows(scores)


if __name__ == "__main__":
    main()
