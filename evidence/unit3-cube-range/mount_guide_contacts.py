"""Disable pairs of dock guides on an otherwise identical recorded reverse replay."""
import csv
import io
import json
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from recorded_mount import ROOT, read_trial, table


def main():
    trials = [read_trial(c, o) for c, o in zip(table("return-scores.csv"), table("mount-target-inputs.csv"), strict=True)]
    inputs = json.dumps([trial for trial, _ in trials])
    export = """import {rearEnvelope} from './rear_envelope.mjs'; import {readText} from '../../web/test/helpers.js';
    process.stdout.write(rearEnvelope(readText('public/robot/vector2/vector-free-dock.xml'),'driven_rear_envelope'));"""
    source = subprocess.check_output(["node", "--input-type=module", "-e", export], cwd=ROOT, text=True)
    fig, axes = plt.subplots(1, 5, figsize=(14, 3.7), sharey=True, layout="constrained")
    for ax, (trial, real) in zip(axes, trials, strict=True):
        t, _, _, yaw, _ = real
        ax.plot(t[t < 0], yaw[t < 0] - yaw[0], color="black", label="Recorded SDK", lw=2)
        ax.set_title(f"Return {trial['name']}")
        ax.set_xlabel("Time from first on-report (s)")
    scores = []
    for label, removed in [("All guides", []), ("Funnels off", ["funnel"]), ("Rails off", ["rail"]),
                           ("Both off", ["funnel", "rail"])]:
        model = ET.fromstring(source)
        for pair in removed:
            for side in ["l", "r"]:
                geom = model.find(f".//geom[@name='dock_{pair}_{side}']")
                assert geom is not None and geom.get("contype") == "0" and geom.get("conaffinity") == "1"
                geom.set("conaffinity", "0")
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "model.xml"
            ET.ElementTree(model).write(path, encoding="utf-8")
            output = subprocess.check_output(["node", str(ROOT/"recorded_mount.mjs"), str(path)], input=inputs, text=True)
        sim = np.genfromtxt(io.StringIO(output), names=True, delimiter=",", dtype=None, encoding="utf8")
        for ax, (trial, real) in zip(axes, trials, strict=True):
            run = sim[sim["session"] == trial["name"]]
            yaw = np.rad2deg(np.unwrap(np.deg2rad(run["yaw_deg"]))) - run["yaw_deg"][0]
            t = real[0][real[0] < 0]
            sampled = np.interp(t, run["t_s"], yaw)
            ax.plot(t, sampled, label=label, lw=1.2, ls="--" if label == "Both off" else "-")
            contact = run["t_s"][run["on"] == 1]
            scores.append([trial["name"], label, sampled[-1], contact[0] if contact.size else "",
                           np.interp(t[-1], run["t_s"], run["out_mm"])])
    axes[0].legend(fontsize=8)
    axes[0].set_ylabel("Yaw change from backup start (°)")
    fig.suptitle("Dock guide contacts during five recorded reverse approaches · driven rear tread support")
    fig.savefig(ROOT/"mount-guide-contacts.png", dpi=160)
    with (ROOT/"mount-guide-contacts.csv").open("w") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["session", "guide_condition", "last_off_yaw_change_deg", "first_contact_s", "last_off_out_mm"])
        writer.writerows(scores)


if __name__ == "__main__":
    main()
