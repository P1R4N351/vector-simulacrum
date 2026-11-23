"""Separate initial lateral offset and heading effects using the five recorded wheel profiles."""
import csv
import io
import json
import subprocess

import matplotlib.pyplot as plt
import numpy as np

from recorded_mount import ROOT, read_trial, table


def main():
    cases = ["Recorded", "Yaw zero", "Offset zero", "Both zero"]
    inputs = []
    for contact, observation in zip(table("return-scores.csv"), table("mount-target-inputs.csv"), strict=True):
        trial, _ = read_trial(contact, observation)
        for index in range(4):
            inputs.append(dict(trial, name=f"{trial['name']}{index // 2}{index % 2}",
                               lat=0 if index >= 2 else trial["lat"], yaw=0 if index % 2 else trial["yaw"]))
    output = subprocess.check_output(["node", str(ROOT / "recorded_mount.mjs")], input=json.dumps(inputs), text=True)
    data = np.genfromtxt(io.StringIO(output), names=True, delimiter=",", dtype=None, encoding="utf8")
    plot(data, cases)


def plot(data, cases):
    fig, axes = plt.subplots(1, 5, figsize=(14, 4.5), sharex=True, sharey=True, layout="constrained")
    with (ROOT / "mount-alignment.csv").open("w") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["session", "variant", "initial_alignment", "first_contact_s", "lat_at_35_mm", "yaw_at_35_deg"])
        for name, ax in zip("ABCDE", axes):
            for index, case in enumerate(cases):
                key = f"{name}{index // 2}{index % 2}"
                for variant in ["canonical", "driven_rear_envelope"]:
                    run = data[(data["session"] == key) & (data["variant"] == variant)]
                    contact = np.flatnonzero(run["on"])
                    entry = run[run["out_mm"] <= 35]
                    writer.writerow([name, variant, case, run["t_s"][contact[0]] if contact.size else "",
                                     *([entry["lat_mm"][0], entry["yaw_deg"][0]] if len(entry) else ["", ""])])
                    if variant == "driven_rear_envelope":
                        approach = run[:contact[0]+1] if contact.size else run
                        line, = ax.plot(approach["lat_mm"], approach["out_mm"], label=case, lw=1.4)
                        ax.plot(approach["lat_mm"][-1], approach["out_mm"][-1], "o" if contact.size else "x",
                                color=line.get_color(), ms=6)
            ax.axhline(0, color="gray", lw=.8)
            ax.axhline(35, color="gray", ls=":", lw=.8)
            ax.set_title(f"Return {name}")
            ax.set_xlabel("Lateral (mm)")
    axes[0].set_ylabel("Outward from dock lip (mm)")
    axes[0].legend(fontsize=8)
    fig.suptitle("Recorded wheel profiles with driven rear support\nCircle = first pad contact   × = replay ended   ··· = 35 mm approach station")
    fig.savefig(ROOT / "mount-alignment.png", dpi=160)


if __name__ == "__main__":
    main()
