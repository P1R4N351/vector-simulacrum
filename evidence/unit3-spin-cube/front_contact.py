"""Replay using the fork and only the two canonical front tread supports."""

import copy
import csv
import json
import subprocess
import numpy as np
import mount_replay as m
from earlier_inputs import TRIALS
from tap_timing import events


def main():
    v, derived, source = m._load_model_config()
    source = copy.deepcopy(source)
    chassis = source.find(".//body[@name='chassis']")
    for geom in chassis.findall("geom"):
        if geom.get("name") not in ("tip_front_l", "tip_front_r"):
            chassis.remove(geom)
    for wheel in chassis.findall("body"):
        if wheel.get("name", "").startswith("wheel_"):
            for geom in wheel.findall("geom"):
                wheel.remove(geom)
    model = m.build_model(source, v, derived, drive="freejoint", contact="full")
    path, poses, counters = [
        m._load_structured(f"earlier-{key}.csv.gz")
        for key in ("path", "pose", "packets")
    ]
    with (m.HERE / "front-contact.csv").open("w") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(
            "trial phase_ms measured_count predicted_count first_error_ms".split()
        )
        for name in TRIALS:
            px = counters[counters["trial"] == name]
            real = events(px["t_s"], px["tap"])
            traj = m.compute_trajectory(
                path[path["trial"] == name], v, model.opt.timestep
            )
            sim = m.simulate_trial(
                model, poses[poses["trial"] == name][0], traj, drive="freejoint"
            )
            for phase in range(0, 10, 2):
                i = round(phase / (1000 * sim["dt"]))
                arg = dict(
                    dt=sim["dt"], start=sim["times"][i], forces=sim["forces"][i:]
                )
                arr = np.array(
                    json.loads(
                        subprocess.check_output(
                            ["node", str(m.HERE / "mount_sensor.mjs")],
                            input=json.dumps(arg),
                            text=True,
                        )
                    )
                )
                pred = events(arr[:, 0], arr[:, 4])
                writer.writerow(
                    [
                        name,
                        phase,
                        len(real),
                        len(pred),
                        round((pred[0] - real[0]) * 1000, 1) if len(pred) else "",
                    ]
                )


if __name__ == "__main__":
    main()
