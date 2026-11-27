"""Compare nine fixed chip positions using identical wheelstand motion and chip clocks."""
import csv
import itertools
import sys
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np

from impact_slots import here, replay, signed, window

POSITIONS = [(0, 0, 0), *itertools.product((-15, 15), repeat=3)]


def insert_sites(tree):
    body = tree.find(".//site[@name='cube_chip']/..")
    attributes = body.find("site[@name='cube_chip']").attrib
    for i, pos in enumerate(POSITIONS):
        name = f"probe{i}"
        ET.SubElement(body, "site", dict(attributes, name=name, pos=" ".join(str(x / 1000) for x in pos)))
        ET.SubElement(tree.find("sensor"), "accelerometer", name=name, site=name)


def matched_slots(model, v, tr, start, onset, peak_time, peak_slot):
    dt = model.opt.timestep
    chips = [replay.CubeAccelerometer(v, dt * 4, seed=17) for _ in POSITIONS]
    step = 0
    reference = []

    def collect(run, t):
        nonlocal step
        step += 1
        if step % 4 == 0:
            for i, chip in enumerate(chips):
                force = run.d.sensor(f"probe{i}").data / (v["gravity_mmps2"] * .001)
                chip.step(step * dt, force)
        if not reference:
            reference.append(run.sensor)

    replay.run(model, v, tr, start, onset, "servo:30", 17, on_step=collect)
    assert chips[0].messages == reference[0].messages
    baseline = np.array([0, 0, v["cube_accel_scale"][2]]) + np.array(v["cube_accel_bias_mg"]) * .001
    slots = []
    for chip in chips:
        times = np.array([start + t for t, _ in chip.messages])
        index = np.argmin(abs(times - peak_time)) * 3 + peak_slot
        words = np.concatenate([np.frombuffer(p, dtype="<i2", offset=2).reshape(3, 3) for _, p in chip.messages])
        slots.append(window(signed(words / 8192, baseline), index))
    return np.array(slots)


def main(fixture):
    v = replay.params.values(replay.params.load(model="vector_2"), {"tip_supports_driven": 0})
    model = replay.build(v, fixture, edit_tree=insert_sites)
    model.opt.timestep = v["sim_timestep_ms"] * .001 / 4
    assert v["cube_chip_position_mm"] == [0, 0, 0]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), layout="constrained", sharey=True)
    rows = []
    for ax, name in zip(axes, ("W4", "W8")):
        tr = replay.Trial(name)
        t = tr.cube["sec"] - tr.zero
        words = np.stack([tr.cube[k] for k in "xyz"], axis=1) / 8192
        obs = signed(words, np.median(words[(t >= -.8) & (t < 0)], axis=0))
        valid = np.flatnonzero((t >= tr.late - .14) & (t <= tr.late + .8))
        peak = valid[np.argmax(abs(obs[valid]))]
        target = window(obs, peak)
        cases = np.stack([matched_slots(model, v, tr, release + tr.shift, onset, t[peak], peak % 3)
                          for release, onset in itertools.product((3.5, 3.8), (tr.early, tr.late))])
        error = np.mean(abs(cases - target), axis=(0, 2))
        if name == "W4":
            selected = int(np.argmin(error))
        for i, pos in enumerate(POSITIONS):
            rows.append([name, *pos, i == selected, error[i], cases[:, i, 4].min(), cases[:, i, 4].max(), target[4]])
        print(name, POSITIONS[selected], error[[0, selected]], cases[:, :, 4].min(), cases[:, :, 4].max(), target[4])
        x = np.arange(-4, 5)
        ax.fill_between(x, cases.min(axis=(0, 1)), cases.max(axis=(0, 1)), color=".86", label="All tested sites and timings")
        ax.plot(x, cases[:, 0].mean(axis=0), "o--", label="Centre · mean of four timings")
        ax.plot(x, cases[:, selected].mean(axis=0), "s--", label="W4-selected site · mean")
        ax.plot(x, target, "ko-", label="Physical cube")
        ax.set(title=name, xlabel="Retained sample relative to physical peak")
    axes[0].set_ylabel("Signed acceleration from rest (g)")
    axes[0].legend(fontsize=8)
    fig.savefig(here / "impact-location.png", dpi=160)
    with (here / "impact-location.csv").open("w") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow("run x_mm y_mm z_mm selected_on_W4 nine_slot_mae_g peak_slot_min_g peak_slot_max_g actual_g".split())
        writer.writerows(rows)


if __name__ == "__main__":
    main(sys.argv[1])
