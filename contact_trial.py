"""42 mm tread-contact experiment at the stock axle locations.

Run e.g. python contact_trial.py replay, spin or tip. The web exporter
uses the same transform on the normal obstacle scene for ?contact=42.
"""
import argparse
import csv
import math
from pathlib import Path
import runpy
from statistics import fmean
import sys
from unittest.mock import patch
from xml.etree import ElementTree as ET

from simulacrum import mjcf


def contact_model(xml, values, previous=False):
    """Move the load-bearing patches, leaving stock joints and tip supports in place."""
    root = ET.fromstring(xml)
    # Each rolling patch already represents a centre of pressure. Additional
    # cylinder/box manifold points made the same flat-table spin depend on the
    # table box's azimuth. Use single-contact CCD for this experimental scene.
    root.find("option/flag").set("multiccd", "enable" if previous else "disable")
    chassis = root.find(".//body[@name='chassis']")
    for side in ("l", "r"):
        for end in ("front", "rear"):
            support = chassis.find(f"geom[@name='tip_{end}_{side}']")
            if support is not None:
                chassis.remove(support)
    inset = (values["wheel_base_mm"] - 42.0) * mjcf.MM / 2
    # The auxiliary shaft is a numerical belt follower. Allow its equality
    # constraint to settle over 10 ms; the real axle and motor use the
    # original parameters. The old minimum was two integration steps.
    min_settle_s = 2 * values["sim_timestep_ms"] / 1000
    settle_s = min_settle_s if previous else max(0.010, min_settle_s)
    for suffix in ("fl", "fr", "rl", "rr"):
        name = "wheel_" + suffix
        axle = chassis.find(f"body[@name='{name}']")
        tread = axle.find("geom")
        shaft = axle.find("joint")
        position = [float(c) for c in axle.get("pos").split()]
        position[0] = 0.0 if suffix[0] == "f" else -values["wheel_base_mm"] * mjcf.MM
        axle.set("pos", mjcf.vec(position))
        position[0] += -inset if suffix[0] == "f" else inset
        roller = ET.SubElement(chassis, "body", name="patch_" + suffix, pos=mjcf.vec(position))

        # Divide the existing reflected rotor inertia between two coupled shafts.
        # A nearly inertia-free follower slips numerically in the soft constraint.
        armature = float(shaft.get("armature"))
        shaft.set("armature", str(0.9 * armature))
        ET.SubElement(roller, "joint", name="patch_" + suffix, type="hinge", axis="0 1 0",
                      armature=str(0.1 * armature))
        contact = ET.fromstring(ET.tostring(tread))
        contact.attrib.update(name="wheel_patch_" + suffix, mass="0.000001")
        roller.append(contact)

        # Retain all but 1 mg of wheel mass at the physical axle. The relieved
        # outer tread takes over as the robot tips or meets a table edge.
        inertia = ET.fromstring(ET.tostring(tread))
        inertia.attrib.update(name="mass_" + suffix, contype="0", conaffinity="0",
                              mass=str(float(tread.get("mass")) - 0.000001))
        axle.append(inertia)
        tread.attrib.update(mass="0", size=str(float(tread.get("size")) - 0.0003))
        ET.SubElement(root.find("equality"), "joint", joint1="patch_" + suffix, joint2=name + "_joint",
                      polycoef="0 1 0 0 0", solref=f"{settle_s} 1")
    return ET.tostring(root, encoding="unicode")


def resting_up_vector(states, segment, values):
    """Apparent vertical in the starting chassis frame from the preceding stationary IMU samples."""
    rows = [r for r in states if segment.t_cmd - 0.9 <= r["t"] < segment.t_cmd
            and r["wl"] == r["wr"] == 0 and math.dist(r["gyro"], (0, 0, 0)) < 0.05]
    if len(rows) < 5:
        raise ValueError(f"{segment.label} lacks a stationary IMU window")
    samples = []
    for row in rows:
        x, y, z = [a / values["gravity_mmps2"] - bias / 1000
                   for a, bias in zip(row["accel"], values["accel_bias_mg"])]
        angle = row["head"] + math.radians(values["imu_pitch_offset_deg"])
        samples.append((x * math.cos(angle) - z * math.sin(angle), y,
                        x * math.sin(angle) + z * math.cos(angle)))
    up = tuple(map(fmean, zip(*samples)))
    return tuple(a / math.hypot(*up) for a in up)


def write_spin_trace(label, rest_tilt=False):
    """CSV in stock sample time, using the same recording and replay as the score."""
    from simulacrum import params, recording, replay
    values = params.values(params.load())
    states, commands = recording.load()
    segment = recording.segments(commands, values)[label]
    base = replay.replay(segment, values, states)["trace"]
    original = mjcf.build_mjcf
    traces = []
    for previous in (True, False):
        def build(v, obstacles=()):
            return contact_model(original(v, obstacles), v, previous=previous)
        with patch.object(mjcf, "build_mjcf", build):
            traces.append(replay.replay(segment, values, states)["trace"])
    if rest_tilt:
        up = resting_up_vector(states, segment, values)
        def inclined(v, obstacles=()):
            root = ET.fromstring(contact_model(original(v, obstacles), v))
            root.find("option").set("gravity", mjcf.vec([-a * v["gravity_mmps2"] * mjcf.MM for a in up]))
            return ET.tostring(root, encoding="unicode")
        with patch.object(mjcf, "build_mjcf", inclined):
            tilted = replay.replay(segment, values, states)["trace"]
    writer = csv.writer(sys.stdout)
    writer.writerow(("seconds", "stock_gyro_z_dps", "default_gyro_z_dps", "previous_42_gyro_z_dps",
                     "revised_42_gyro_z_dps", "stock_left_mmps", "stock_right_mmps",
                     "revised_left_mmps", "revised_right_mmps") + (("inclined_42_gyro_z_dps",) if rest_tilt else ()))
    for i, ((t, default, stock), (_, old, _), (_, revised, _)) in enumerate(zip(base, *traces)):
        writer.writerow((t, math.degrees(stock["gyro"][2]), default["gyro_z_dps"], old["gyro_z_dps"],
                         revised["gyro_z_dps"], stock["wl"], stock["wr"], revised["wl"], revised["wr"])
                        + ((tilted[i][1]["gyro_z_dps"],) if rest_tilt else ()))


if __name__ == "__main__":
    targets = {"replay": "replay_recording.py", "tip": "wheelstand_check.py", "spin": "diagnose_turns.py"}
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("check", choices=[*targets, "trace"])
    parser.add_argument("--rest-tilt", action="store_true", help="include a trace using the pre-command IMU vertical")
    args, remaining = parser.parse_known_args()
    if args.check == "trace":
        if len(remaining) > 1:
            parser.error("trace takes one optional recorded spin label")
        write_spin_trace(remaining[0] if remaining else "spin_ccw_100", args.rest_tilt)
        sys.exit(0)
    if args.rest_tilt:
        parser.error("--rest-tilt applies to trace")
    original = mjcf.build_mjcf
    script = Path(__file__).with_name(targets[args.check])
    sys.argv = [str(script), *remaining]
    with patch.object(mjcf, "build_mjcf", lambda v, obstacles=(): contact_model(original(v, obstacles), v)):
        runpy.run_path(str(script), run_name="__main__")
