"""Action-delimited chassis response during stock carries and empty raised-fork controls."""

import csv, gzip, json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

here = Path(__file__).resolve().parent
out = []
fig, axes = plt.subplots(4, 3, sharex="col", figsize=(12, 8))
for archive in sorted(here.glob("rotation-*.jsonl.gz")):
    events = list(map(json.loads, gzip.open(archive, "rt")))
    states = [e for e in events if e.get("name") == "robot_state"]
    for i, e in enumerate(events):
        action = e.get("action", "")
        if not action.startswith(("empty ", "loaded ")):
            continue
        load, name = action.split(" ", 1)
        physical = (
            "false carry"
            if load == "loaded" and "false-carry" in archive.name
            else load
        )
        if not (
            (e["kind"] == "begin" and name.startswith("turn "))
            or (e["kind"] == "pulse_begin" and name == "reverse")
        ):
            continue
        trial = next(x["number"] for x in reversed(events[:i]) if x["kind"] == "trial")
        reversing = name == "reverse"
        end_kind = "pulse_end" if reversing else "end"
        end_name = load + " forward2" if reversing else action
        end = next(
            (
                x["t"]
                for x in events[i + 1 :]
                if x["kind"] == end_kind and x.get("action") == end_name
            ),
            None,
        )
        if end is None:
            continue
        start = e["t"]
        sel = [
            x
            for x in states
            if start - 0.2 <= x["t"] <= end + (0.45 if reversing else 0)
        ]
        t = np.array([x["t"] - start for x in sel])
        d = [x["data"] for x in sel]

        def col(key):
            return np.array([v.get(key, 0) for v in d])

        yaw = np.rad2deg(np.unwrap(col("pose_angle_rad")))
        pitch = np.rad2deg(col("pose_pitch_rad"))
        lift = col("lift_height_mm")
        pre = t < 0
        if not pre.any():
            continue
        h = lift - np.median(lift[pre])
        theta = pitch - np.median(pitch[pre])
        yaw -= np.median(yaw[pre])
        l, r = col("left_wheel_speed_mmps"), col("right_wheel_speed_mmps")
        speed = (l + r) / 2 if reversing else (r - l) / 2
        during = t >= 0
        out.append(
            [
                archive.name,
                trial,
                physical,
                name,
                len(t[during]),
                end - start,
                yaw[-1],
                np.ptp(theta[during]),
                max(0, -h[during].min()),
                np.percentile(lift[pre], 50),
                speed[during].min(),
                speed[during].max(),
                sum(
                    v.get("carrying_object_id", -1) == 1
                    for j, v in enumerate(d)
                    if during[j]
                ),
            ]
        )
        j = 2 if reversing else int(name == "turn back")
        for ax, val in zip(axes[:, j], (speed, yaw, theta, h)):
            ax.plot(
                t,
                val,
                ".-",
                ms=2,
                lw=1,
                alpha=0.85,
                color={
                    "loaded": "#00778a",
                    "empty": "#c4712d",
                    "false carry": "#777777",
                }[physical],
            )
        axes[0, j].set_title("Rapid reversals" if reversing else name)
        if reversing:
            for x in events[i:]:
                if x["t"] > end:
                    break
                if x["kind"] == "pulse_begin":
                    axes[0, j].plot(x["t"] - start, x["left"], "k_", ms=6)
for a in axes[-1]:
    a.set_xlabel("Seconds from first command")
for a, label in zip(
    axes[:, 0],
    (
        "Wheel speed* (mm/s)",
        "Yaw change (°)",
        "Chassis pitch change (°)",
        "Lift change (mm)",
    ),
):
    a.set_ylabel(label)
fig.suptitle(
    "Physical Unit 3 · teal verified carry, orange empty raised fork, gray false carry"
)
fig.tight_layout()
fig.savefig(here / "rotation.png", dpi=145)
with (here / "rotation.csv").open("w") as f:
    w = csv.writer(f, lineterminator="\n")
    w.writerow(
        [
            "source",
            "trial",
            "condition",
            "action",
            "states",
            "action_seconds",
            "yaw_delta_deg",
            "pitch_range_deg",
            "lift_drop_mm",
            "start_lift_mm",
            "min_wheel_mode_mmps",
            "max_wheel_mode_mmps",
            "states_carry_id1",
        ]
    )
    w.writerows(out)
