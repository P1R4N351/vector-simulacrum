"""Replay a recorded turn with measured front-shaft speeds and a free chassis."""
import argparse
import csv
import json
import math
from pathlib import Path
from unittest.mock import patch
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

from contact_trial import contact_model
from simulacrum import mjcf, params, replay
from simulacrum.robot import VectorSim


def input_rows(path):
    events = [json.loads(line) for line in path.read_text().splitlines()]
    command = next(r for r in events if r.get("name") == "wheels" and r.get("label") == "spin_ccw_100")
    start = command["t"]
    stop = next(r["issued_t"] for r in events if r.get("name") == "stop" and r["t"] > start) - start
    rows = [r for r in events if r["k"] == "s" and -.40 <= r["t"]-start <= stop+.65]
    time = np.array([r["t"]-start for r in rows])
    wheels = np.array([[r["wl"], r["wr"]] for r in rows])
    gyro = np.degrees([r["gyro"][2] for r in rows])
    return rows, time, wheels, gyro, stop


def run(rows, time, wheels, span, *, dense=False, include_pose=False, configure=None, observe=None,
        step_ms=None):
    values = dict(params.values(params.load(model="vector_2")), **replay.OPEN_TABLE)
    builder = mjcf.build_mjcf

    def build(v, obstacles=()):
        root = ET.fromstring(contact_model(builder(v, obstacles), v))
        if configure is not None:
            configure(root)
        # At 48 mm use precisely the same contact solver and follower joints.
        if span == 48:
            for suffix in ("fl", "fr", "rl", "rr"):
                axle = root.find(f".//body[@name='wheel_{suffix}']")
                root.find(f".//body[@name='patch_{suffix}']").set("pos", axle.get("pos"))
        for side in ("left", "right"):
            motor = root.find(f"actuator/general[@name='{side}_wheel']")
            motor.set("gainprm", "1")
            motor.set("biastype", "none")
            motor.set("ctrllimited", "false")
        return ET.tostring(root, encoding="unicode")

    with patch.object(mjcf, "build_mjcf", build):
        sim = VectorSim(values=values, head_deg=math.degrees(rows[0]["head"]), lift_mm=rows[0]["lift"])
    sim.step(round(replay.PRE_S/sim.dt))
    m, d = sim.model, sim.data
    if step_ms is not None:
        # Keep the settled initial state and all physical constraint times fixed.
        m.opt.timestep = step_ms / 1000
    motors = [m.actuator(side+"_wheel").id for side in ("left", "right")]
    gyro = m.sensor("gyro").adr.item()+2
    dt = m.opt.timestep
    clock = time[0] + np.arange(math.ceil((time[-1]-time[0])/dt)+1)*dt
    desired = np.column_stack([np.interp(clock, time, wheel) for wheel in wheels.T])
    output = np.empty((len(clock), 6 if include_pose else 3))
    # Solve only for the two motor torques needed for the next encoder sample.
    # Save/restore all integration state so trial solves never advance the clock.
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    saved = np.empty(mujoco.mj_stateSize(m, spec))

    def advance(torque):
        mujoco.mj_setState(m, d, saved, spec)
        d.ctrl[motors] = torque
        mujoco.mj_step(m, d)
        return np.asarray(sim._wheel_speeds())

    for i in range(len(clock)):
        # Sensor kinematics after the integration step use the updated velocity.
        mujoco.mj_forward(m, d)
        output[i, :3] = [math.degrees(d.sensordata[gyro]), *sim._wheel_speeds()]
        if include_pose:
            output[i, 3:] = [*sim.pose_mm()[:2], sim._yaw()]
        if observe is not None:
            observe(m, d, clock[i])
        if i+1 < len(clock):
            mujoco.mj_getState(m, d, saved, spec)
            torque = d.ctrl[motors].copy()
            for attempt in range(12):
                actual = advance(torque)
                error = desired[i+1] - actual
                if np.max(abs(error)) < .01:
                    break
                perturb = 1e-5
                jac = np.column_stack([(advance(torque + axis*perturb) - actual)/perturb
                                       for axis in np.eye(2)])
                torque += np.linalg.solve(jac, error)
            else:
                raise RuntimeError(f"wheel constraint failed at {clock[i]:.3f} s")
    if dense:
        return clock, output
    return np.column_stack([np.interp(time, clock, col) for col in output.T])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()
    rows, t, wheels, obs, stop = input_rows(args.recording)
    # Steady starts at the first sample where both encoders reach 90% of command.
    steady = t[np.flatnonzero(np.min(abs(wheels), axis=1) >= 90)[0]]
    phase = np.select([t<0, t<steady, t<stop], ["rest", "spin-up", "steady"], default="stopping")
    columns = [t, phase, *wheels.T, obs]
    header = ["time_s", "phase", "left_mmps", "right_mmps", "gyro_dps"]
    for span in (42, 48):
        result = run(rows, t, wheels, span)
        columns.extend(result.T)
        header.extend(f"{name}_{span}" for name in ("gyro_dps", "left_mmps", "right_mmps"))
        print(f"{span} mm, max wheel tracking error {np.max(abs(result[:,1:]-wheels)):.3f} mm/s")
        for label in ("spin-up", "steady", "stopping"):
            pick = phase == label
            delta = result[pick,0] - obs[pick]
            print(label, pick.sum(), "obs, sim, bias, RMS", *(f"{v:.2f}" for v in (
                obs[pick].mean(), result[pick,0].mean(), delta.mean(), np.sqrt(np.mean(delta**2)))))
    with args.csv.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(zip(*columns))


if __name__ == "__main__":
    main()
