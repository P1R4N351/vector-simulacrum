"""Compare measured angular speed with MuJoCo's tread contact moments in recorded spins.

Run from this directory with the simulacrum's Python environment. The optional
contact inset moves the four wheel proxies in memory for a sensitivity test.
The default inset moves wheel mass as well. --preserve-inertia compensates for
that change. The production wheelbase stays at 48 mm.
"""
import argparse
import math
from pathlib import Path
from unittest.mock import patch
from xml.etree import ElementTree as ET

import mujoco
import numpy as np

from simulacrum import mjcf, params, recording, replay
from simulacrum.robot import VectorSim


def tread_moments(sim, chassis):
    """Signed drive and lateral scrub moments about the robot COM, in mN m."""
    data = sim.data
    basis = data.xmat[chassis].reshape(3, 3)
    moment = np.zeros(2)
    for i in range(data.ncon):
        contact = data.contact[i]
        names = [sim.model.geom(int(g)).name or "" for g in contact.geom]
        if not any(name.startswith("wheel_") for name in names):
            continue
        wrench = np.zeros(6)
        mujoco.mj_contactForce(sim.model, data, i, wrench)
        # MuJoCo gives the force on geom 2 in the contact frame.
        sign = 1 if names[1].startswith("wheel_") else -1
        force = sign * basis.T @ contact.frame.reshape(3, 3).T @ wrench[:3]
        arm = basis.T @ (contact.pos - data.subtree_com[chassis])
        moment += [-arm[1] * force[0], arm[0] * force[1]]
    return moment * 1000


def steady_spin(segment, states, values):
    start = next(row for row in reversed(states) if row["t"] <= segment.t_cmd)
    sim = VectorSim(values=dict(values, **replay.OPEN_TABLE),
                    head_deg=math.degrees(start["head"]), lift_mm=start["lift"], noise_seed=0)
    chassis = sim.model.body("chassis").id
    gyro = sim.model.sensor("gyro").adr.item()
    duration = segment.t_stop - segment.t_cmd
    rows = [r for r in states if segment.t_cmd + duration / 2 <= r["t"] <= segment.t_stop]
    measured = np.mean([[math.degrees(np.linalg.norm(r["gyro"])),
                         (abs(r["wl"]) + abs(r["wr"])) / 2] for r in rows], axis=0)
    sim.step(round(replay.PRE_S / sim.dt))
    sim.set_wheel_motors(segment.left, segment.right)
    ticks = []
    for _ in range(round(duration / sim.dt)):
        sim.step(1)
        state = sim.reported_now()
        ticks.append([math.degrees(np.linalg.norm(sim.data.sensordata[gyro:gyro + 3])),
                      (abs(state["wl"]) + abs(state["wr"])) / 2, *tread_moments(sim, chassis)])
    # Read at the same times as the hardware. Each state's gyro and wheel
    # speeds come from that row, never from the odometry yaw field.
    sampled = [ticks[min(len(ticks) - 1, max(0, round((r["t"] - segment.t_cmd) / sim.dt) - 1))]
               for r in rows]
    return measured, np.mean(sampled, axis=0)


def compensate_wheel_inset(xml, values, inset_mm):
    """Preserve the robot inertia tensor while shortening the contact span.

    The symmetric shifts of equal wheel masses leave total COM unchanged.
    Restore the parallel-axis contribution lost as the wheels move inward.
    The wheel tensors about their own spin axes are unchanged.
    """
    root = ET.fromstring(xml)
    chassis = root.find(".//body[@name='chassis']/inertial")
    inertia = np.diag(np.fromstring(chassis.attrib.pop("diaginertia"), sep=" "))
    mass = values["wheel_mass_g"] * 0.001
    for name in ("fl", "fr", "rl", "rr"):
        body = root.find(f".//body[@name='wheel_{name}']")
        moved = np.fromstring(body.attrib["pos"], sep=" ")
        original = moved.copy()
        original[0] += (1 if name.startswith("f") else -1) * inset_mm * 0.001
        for position, sign in ((original, 1), (moved, -1)):
            inertia += sign * mass * (np.eye(3) * (position @ position) - np.outer(position, position))
    chassis.set("fullinertia", " ".join(str(inertia[i, j]) for i, j in
                ((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2))))
    return ET.tostring(root, encoding="unicode")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inset-mm", type=float, default=0.0, help="move each support toward the belt midpoint")
    parser.add_argument("--com-x-mm", type=float, help="override the assumed fore-aft centre of mass")
    parser.add_argument("--recording", type=Path, default=Path(__file__).resolve().parent /
                        "evidence/hw-rec-20261118/run-e-spins.jsonl")
    parser.add_argument("--preserve-inertia", action="store_true",
                        help="restore the original whole-robot inertia tensor after moving the supports")
    args = parser.parse_args()
    values = params.values(params.load())
    if not 0 <= args.inset_mm < values["wheel_base_mm"] / 2:
        parser.error("inset must lie between zero and half the wheelbase")
    if args.com_x_mm is not None:
        values["com_x_mm"] = args.com_x_mm
    wheel_xml = mjcf._wheel_xml

    def proxy(v, name, x, y, radius, driven):
        shift = -args.inset_mm if driven else args.inset_mm
        return wheel_xml(v, name, x + shift, y, radius, driven)

    build_model = mjcf.build_mjcf

    def diagnostic_model(v, obstacles=()):
        xml = build_model(v, obstacles)
        if args.preserve_inertia and args.inset_mm:
            return compensate_wheel_inset(xml, v, args.inset_mm)
        return xml

    states, commands = recording.load(args.recording)
    print("segment,robot_angular_dps,model_angular_dps,robot_wheel_mmps,model_wheel_mmps,"
          "drive_mNm,scrub_mNm")
    with patch.object(mjcf, "_wheel_xml", proxy), patch.object(mjcf, "build_mjcf", diagnostic_model):
        for seg in recording.segments(commands, values).values():
            if seg.kind == "wheels" and seg.left * seg.right < 0:
                robot, model = steady_spin(seg, states, values)
                row = (robot[0], model[0], robot[1], model[1], model[2], model[3])
                print(seg.label + "," + ",".join(f"{n:.2f}" for n in row))


if __name__ == "__main__":
    main()
