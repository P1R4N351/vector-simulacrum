"""Bracket physical encoder and status transitions at their SDK packet cadence."""
import csv
import json
from math import radians
from pathlib import Path
from statistics import median

import matplotlib.pyplot as plt
from simulacrum import mjcf, params

root = Path(__file__).resolve().parent


def records(file):
    with (root / file).open() as stream:
        return list(csv.DictReader(stream))


def first_interval(states, predicate):
    for left, right in zip(states, states[1:]):
        if not predicate(left) and predicate(right):
            return left["t"], right["t"]
    raise ValueError("missing transition")


def main():
    history = records("lift-ack.csv")
    calibration = [r for r in history if r["unit"] == "1" and r["near_target"] == "False"]
    lag = median(float(r["sdk_success_s"]) - float(r["physical_flag_edge_s"]) for r in calibration)
    raw = [json.loads(line) for line in (root / "workshop-validation.jsonl").read_text().splitlines()]
    cues = [r["t"] for r in raw if r.get("name") == "lift"]
    replay = records("workshop-validation.csv")
    assert len(cues) == len(replay)
    v = params.values(params.load(model="vector_1"))
    tol = radians(v["lift_angle_tol_deg"])
    fig, ax = plt.subplots(figsize=(9, 4.5), layout="constrained")
    rows = []
    for i, (t, result) in enumerate(zip(cues, replay)):
        sdk = float(result["sdk_s"])
        sim_flag = float(result["revised_s"])
        sim_entry = float(result["earlier_s"])
        angle = mjcf.lift_angle_of_height(v, float(result["target_mm"]))
        states = [r for r in raw if r["k"] == "s" and t <= r["t"] <= t + sdk + .10]
        entry = first_interval(states, lambda r: abs(mjcf.lift_angle_of_height(v, r["lift"]) - angle) <= tol)
        flag = first_interval(states, lambda r: bool(r["status"] & 256))
        for offset, label, color, interval, sim in [
            (-.14, "Encoder enters tolerance", "C0", entry, sim_entry),
            (0, "Lift in position", "C1", flag, sim_flag)
        ]:
            lo, hi = [x - t for x in interval]
            ax.plot([lo, hi], [i+offset]*2, color=color, linewidth=5, alpha=.55, label=label if i==0 else None)
            ax.scatter(sim, i+offset, color=color, marker="|", s=170)
        ax.scatter(sdk, i+.14, color="black", marker="o", label="SDK success" if i==0 else None)
        ax.scatter(sim_flag+lag, i+.14, color="black", marker="x", label="Predicted SDK success" if i==0 else None)
        rows.append([i+1, float(result["target_mm"]), sim_entry, *[x-t for x in entry], sim_flag,
                     *[x-t for x in flag], sdk, sim_flag+lag])
    with (root / "lift-residual.csv").open("w") as stream:
        out = csv.writer(stream, lineterminator="\n")
        out.writerow("move target_mm sim_entry_s entry_lo_s entry_hi_s sim_settled_s flag_lo_s flag_hi_s sdk_s predicted_sdk_s".split())
        out.writerows(rows)
    ax.set(yticks=list(range(6)), yticklabels=["Raise 1","Lower 1","Raise 2","Lower 2","Raise 3","Lower 3"],
           xlabel="Seconds from lift command", title="Physical transition intervals and simulated event times")
    ax.invert_yaxis()
    ax.legend(fontsize=8, ncols=2)
    fig.savefig(root / "lift-residual.png", dpi=170)


if __name__ == "__main__":
    main()
