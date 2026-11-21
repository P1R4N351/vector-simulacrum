"""Transfer a frozen unit 3 contact inset to four unit 1 run E spins.

Use the bbec512a simulacrum source on PYTHONPATH, with contact.py on the same
path. Arguments are the run E JSONL, output directory, and frozen inset in mm.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from unittest.mock import patch

import mujoco
import numpy as np
from simulacrum import mjcf, params, recording, replay
from simulacrum.robot import VectorSim


def gyro_axes(head, gyro):
    x, y, z = gyro
    return dict(yaw_rad_s=x * math.sin(head) + z * math.cos(head),
                roll_rad_s=x * math.cos(head) - z * math.sin(head),
                pitch_rad_s=y, gyro_z_rad_s=z, roll_residual_rad_s=x - z * math.tan(head))


def data_row(time, head, gyro, lift, wl, wr, x, y, yaw, imu_offset_deg):
    return dict(t_s=time, **gyro_axes(head + math.radians(imu_offset_deg), gyro), head_deg=math.degrees(head),
                lift_mm=lift, left_mm_s=wl, right_mm_s=wr,
                x_mm=x, y_mm=y, heading_rad=yaw)


def simulate(values, start, seg, inset):
    builder = mjcf.build_mjcf

    def build(*args):
        from contact import contact_xml
        return contact_xml(builder(*args), inset)

    with patch.object(mjcf, "build_mjcf", builder if inset is None else build):
        sim = VectorSim(values=dict(values, **replay.OPEN_TABLE),
                        head_deg=math.degrees(start["head"]), lift_mm=start["lift"], noise_seed=0)
    pre = round(replay.PRE_S / sim.dt)
    active = round((seg.t_stop - seg.t_cmd) / sim.dt)
    post = round(replay.POST_S / sim.dt)
    result = []
    for tick in range(pre + active + post):
        if tick == pre:
            assert sim.set_wheel_motors(seg.left, seg.right)
        if tick == pre + active:
            sim.stop_all_motors()
        state = sim.step(1)
        if not math.isclose(state.time_s, sim.tick * sim.dt, abs_tol=1e-8):
            continue
        result.append(data_row(state.time_s - pre * sim.dt, state.head_angle_rad,
                               state.gyro_radps, state.lift_height_mm, state.left_wheel_speed_mmps,
                               state.right_wheel_speed_mmps, state.x_mm, state.y_mm, state.pose_angle_rad,
                               values["imu_pitch_offset_deg"]))
    return result


def icr(rows):
    # The same body-fixed point as the P2 camera analysis, relative to front axle.
    yaw = np.array([r["heading_rad"] for r in rows])
    lens = -36.3
    x = np.array([r["x_mm"] for r in rows]) + lens * np.cos(yaw)
    y = np.array([r["y_mm"] for r in rows]) + lens * np.sin(yaw)
    cx, cy, _ = np.linalg.lstsq(np.column_stack((2*x, 2*y, np.ones(len(x)))),
                                x*x + y*y, rcond=None)[0]
    return float(lens + np.median((cx-x)*np.cos(yaw) + (cy-y)*np.sin(yaw)))


def score(rows, duration, source):
    steady = [r for r in rows if duration/2 <= r["t_s"] <= duration]
    out = {"n_steady":len(steady), "window_start_s":duration/2, "window_end_s":duration}
    for name in ("yaw_rad_s", "roll_rad_s", "pitch_rad_s", "gyro_z_rad_s", "roll_residual_rad_s"):
        out["mean_abs_"+name] = float(np.mean([abs(r[name]) for r in steady]))
    out["mean_signed_yaw_rad_s"] = float(np.mean([r["yaw_rad_s"] for r in steady]))
    out["simulation_icr_x_mm"] = icr(steady) if source != "unit 1" else None
    return out


def write_csv(path, rows):
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("inset", type=float, nargs="?")
    args = parser.parse_args()
    values = params.values(params.load(model="vector_1"))
    assert values["wheel_base_mm"] == 48
    states, commands = recording.load(args.recording)
    summary, output = [], []
    for seg in recording.segments(commands, values).values():
        if seg.kind != "wheels" or seg.left * seg.right >= 0:
            continue
        start = replay._state_at(states, seg.t_cmd)
        obs = [data_row(r["t"] - seg.t_cmd, r["head"], r["gyro"], r["lift"], r["wl"], r["wr"],
                        *r["pose"][:2], r["yaw"], values["imu_pitch_offset_deg"]) for r in states
               if seg.t_cmd-replay.PRE_S <= r["t"] <= seg.t_stop+replay.POST_S]
        series = {"unit 1":obs, "original":simulate(values, start, seg, None)}
        if args.inset is not None:
            for name, inset in (("proxy_zero",0), ("frozen",args.inset)):
                series[name] = simulate(values, start, seg, inset)
        for name, rows in series.items():
            fixed = dict(spin=seg.label, source=name, command_left_mm_s=seg.left, command_right_mm_s=seg.right,
                         start_head_deg=math.degrees(start["head"]), start_lift_mm=start["lift"])
            summary.append(dict(**fixed, **score(rows, seg.t_stop-seg.t_cmd, name)))
            output.extend(dict(spin=seg.label, source=name, **r) for r in rows)
    assert len(set(r["spin"] for r in summary)) == 4
    args.out.mkdir(exist_ok=True, parents=True)
    write_csv(args.out/"c7_scores.csv", summary)
    write_csv(args.out/"c7_traces.csv", output)
    (args.out/"c7_provenance.json").write_text(json.dumps(dict(
        base_commit="bbec512a121207e01b17210f79017b16e7cffea4", recording_sha256=recording.sha256(args.recording),
        inset_mm=args.inset, mujoco=mujoco.__version__, model="vector_1", nominal_axles_mm=values["wheel_base_mm"],
        solver_step_ms=values["sim_timestep_ms"], state_period_ms=values["state_period_ms"],
        imu_pitch_offset_deg=values["imu_pitch_offset_deg"], steady_window="second half of commanded time",
        sim_states="one per SDK state publication", real_positions="SDK odometry only; independent C7 ICR unavailable",
        contact_helper_sha256=(hashlib.sha256(Path(__import__("contact").__file__).read_bytes()).hexdigest()
                               if args.inset is not None else None)), indent=2)+"\n")


if __name__ == "__main__":
    main()
