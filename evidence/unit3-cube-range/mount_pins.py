"""Measure pin-to-pad offsets in the dock frame through recorded backing profiles."""
import csv
import io
import json
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from recorded_mount import ROOT, read_trial, table


def main():
    trials = [read_trial(c, o)[0] for c, o in zip(table("return-scores.csv"), table("mount-target-inputs.csv"), strict=True)]
    export = """import {rearEnvelope} from './rear_envelope.mjs'; import {readText} from '../../web/test/helpers.js';
    process.stdout.write(rearEnvelope(readText('public/robot/vector2/vector-free-dock.xml'),'driven_rear_envelope'));"""
    source = subprocess.check_output(["node", "--input-type=module", "-e", export], cwd=ROOT, text=True)
    model = ET.fromstring(source)
    for kind in ["rail", "funnel"]:
        for side in ["l", "r"]:
            model.find(f".//geom[@name='dock_{kind}_{side}']").set("conaffinity", "0")

    fig, axes = plt.subplots(2, 5, figsize=(14, 6), sharex="col", sharey=True, layout="constrained")
    scores = []
    for panel, (label, xml) in enumerate([("Guides on", source), ("Guides off", ET.tostring(model, encoding="unicode"))]):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"model.xml"
            path.write_text(xml)
            raw = subprocess.check_output(["node", str(ROOT/"recorded_mount.mjs"), str(path)], input=json.dumps(trials),
                                          env=dict(os.environ, VECTOR_TRACE_DOCK_PINS="1"), text=True)
        data = np.genfromtxt(io.StringIO(raw), names=True, dtype=None, delimiter=",", encoding="utf8")
        for ax, trial in zip(axes[panel], trials, strict=True):
            run = data[data["session"] == trial["name"]]
            pins = np.stack([np.column_stack([run[s+a] for a in "xyz"]) for s in "lr"], axis=1)
            gap = np.maximum(pins - [9, 3.8, 4], [-9, -3.8, -5] - pins)
            np.testing.assert_array_equal(gap.max(axis=(1, 2)) <= 0, run["on"])
            on = np.flatnonzero(run["on"])
            i = on[0] if len(on) else np.argmin(gap.max(axis=(1, 2)))
            scores.append([trial["name"], label, int(bool(len(on))), run["t_s"][i], *pins[i].flat,
                           *gap[i].max(axis=0)])
            for j, axis in enumerate(["Longitudinal", "Lateral", "Vertical"]):
                ax.plot(run["t_s"], gap[:, :, j].max(axis=1), label=axis, lw=1.2)
            ax.axhline(0, color="black", lw=.6)
            ax.axvline(run["t_s"][i], color="gray", ls=":", lw=.9)
            ax.set_xlim(-1.4, .5)
            ax.set_ylim(-8, 55)
            ax.set_title(f"{trial['name']} · {label}")
            if panel:
                ax.set_xlabel("From physical on-report (s)")
    axes[0, 0].legend(fontsize=8)
    for ax in axes[:, 0]:
        ax.set_ylabel("Worst pin excess (mm)")
    fig.suptitle("Distance outside contact tolerance · each axis must reach zero or below at the same tick")
    fig.savefig(ROOT/"mount-pins.png", dpi=160)
    with (ROOT/"mount-pins.csv").open("w") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow("session guides acquired time_s lx ly lz rx ry rz x_excess_mm y_excess_mm z_excess_mm".split())
        writer.writerows(scores)


if __name__ == "__main__":
    main()
