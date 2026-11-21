"""Print the simulacrum's behaviour as numbers..

    the GPU host python measure.py

The figures in README.md come from this script. It measures the model, not
a robot: nothing here is evidence about a real Vector. Nothing is asserted
about the outcomes; they are printed as they are, including the ones that
disagree with the firmware's own figures.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Any, Dict

sys.path.insert(0, str(Path(__file__).resolve().parent))

from simulacrum import params  # noqa: E402
from simulacrum.robot import CONTROL_DEFAULT, CONTROL_OVERRIDE, VectorSim  # noqa: E402

DRIVES = ((200.0, 100.0), (-100.0, 60.0), (50.0, 20.0), (250.0, 160.0))
TURNS = ((90.0, 60.0), (-180.0, 120.0), (30.0, 60.0), (30.0, 15.0))
EDGE_SPEEDS = (50.0, 100.0, 140.0, 150.0, 160.0, 170.0, 220.0)
EDGE_START_MM = 150.0
STALL_TORQUES_MNM = (50.0, 100.0, 200.0)
RECORDED_SPIN_SPEEDS = (30.0, 60.0, 100.0)
COM_X_MM = (-30.0, -24.0, -19.0, -14.0)
LONG_TABLE = {"table_length_mm": 1500.0}


def drives() -> None:
    for dist, speed in DRIVES:
        sim = VectorSim()
        result = sim.drive_straight(dist, speed)
        x, y, _ = sim.pose_mm()
        assert math.isfinite(x) and math.isfinite(y), "pose must be finite"
        print(f"drive {dist:.0f} mm at {speed:.0f} mm/s: {result.result}, x {x:.2f} y {y:.2f} mm, "
              f"heading {math.degrees(sim.heading_total_rad()):.3f} deg, {result.ticks * sim.dt:.2f} s")


def turns(v: Dict[str, Any]) -> None:
    for angle, speed in TURNS:
        sim = VectorSim()
        result = sim.turn_in_place(angle, speed)
        assert math.isfinite(result.turned_deg), "turn must be finite"
        print(f"turn {angle:.0f} deg at {speed:.0f} deg/s: {result.result}, turned {result.turned_deg:.2f} deg, "
              f"{result.ticks * sim.dt:.2f} s")
    sim = VectorSim()
    sim.turn_in_place(180.0)
    x, y, _ = sim.pose_mm()
    print(f"turn centre, from a 180 deg turn: x {0.5 * x:.1f} y {0.5 * y:.1f} mm "
          f"(firmware drive centre offset {v['drive_center_offset_mm']:.0f} mm)")


def spin_rate(values: Dict[str, Any], speed: float) -> float:
    """Turn rate in deg/s once a spin at +-speed has settled."""
    sim = VectorSim(values=values)
    sim.set_wheel_motors(-speed, speed)
    sim.advance(4.0)
    before = sim.heading_total_rad()
    sim.advance(1.0)
    rate = math.degrees(sim.heading_total_rad() - before)
    assert math.isfinite(rate), "spin rate must be finite"
    return rate


def spins(v: Dict[str, Any]) -> None:
    for speed in (50.0, v["wheel_speed_max_mmps"]):
        rate = spin_rate(v, speed)
        ideal = math.degrees(2.0 * speed / v["tread_spacing_mm"])
        print(f"spin at +-{speed:.0f} mm/s, after 4 s: {rate:.1f} deg/s, {rate / ideal:.2f} of the no-slip rate "
              f"{ideal:.1f} (firmware: about {v['body_rotation_speed_observed_dps']:.0f} deg/s at full speed)")


def spin_fraction_against_the_recording(doc: Dict[str, Any]) -> None:
    """The model's turn rate over the no-slip rate at the speeds of the recorded spins, and what com_x does to it."""
    v = params.values(doc)
    real = v["spin_rate_fraction_measured"]
    for speed in RECORDED_SPIN_SPEEDS:
        ideal = math.degrees(2.0 * speed / v["tread_spacing_mm"])
        ccw, cw = spin_rate(v, speed), -spin_rate(v, -speed)
        print(f"spin at +-{speed:.0f} mm/s: model {ccw / ideal:.3f} counter-clockwise, {cw / ideal:.3f} clockwise "
              f"of no-slip; short of the recording's mean {real:.2f} by {(real - ccw / ideal) * ideal:.1f} deg/s")
    print("sensitivity of the spin fraction at +-60 mm/s to com_x_mm (the table's value is the only one used elsewhere):")
    for com_x in COM_X_MM:
        values = params.values(doc, {"com_x_mm": com_x})
        ideal = math.degrees(2.0 * 60.0 / values["tread_spacing_mm"])
        fraction = spin_rate(values, 60.0) / ideal
        assert 0.0 < fraction < 1.5, "spin fraction out of any plausible range"
        print(f"  com_x {com_x:.0f} mm: {fraction:.3f}")


def scatter_in_motion(doc: Dict[str, Any]) -> None:
    """The model's sensor scatter in a held spin, beside the recording's, head at the recorded 22.4 degrees."""
    from simulacrum import recording
    v = params.values(doc)
    real = recording.moving_scatter(*recording.load(), v["gravity_mmps2"])
    for speed in RECORDED_SPIN_SPEEDS:
        sim = VectorSim(values=v, head_deg=22.4, noise_seed=3)
        sim.set_wheel_motors(-speed, speed)
        sim.advance(1.3)
        states = [sim.step(sim._state_ticks) for _ in range(21)]
        spread = lambda xs: math.sqrt(sum((x - sum(xs) / len(xs)) ** 2 for x in xs) / len(xs))
        gyro = max(spread([math.degrees(s.gyro_radps[i]) for s in states]) for i in range(3))
        accel = max(spread([s.accel_mmps2[i] for s in states]) for i in range(3)) * 1000.0 / v["gravity_mmps2"]
        both = [real[f"spin_{way}_{speed:.0f}"] for way in ("ccw", "cw")]
        assert len(states) == 21 and len(both) == 2, "scatter needs 21 states and both recorded spins"
        print(f"scatter in a spin at +-{speed:.0f} mm/s, largest axis: gyro {gyro:.1f} deg/s (recording "
              f"{both[0]['gyro_sd_dps']:.1f} and {both[1]['gyro_sd_dps']:.1f}), accelerometer {accel:.0f} mg "
              f"(recording {both[0]['accel_sd_mg']:.0f} and {both[1]['accel_sd_mg']:.0f})")


def step_and_coast(doc: Dict[str, Any]) -> None:
    for speed in (50.0, 100.0, 160.0):
        sim = VectorSim(values=params.values(doc, LONG_TABLE), start_xy_mm=(-600.0, 0.0))
        sim.set_wheel_motors(speed, speed)
        reached = None
        for tick in range(600):
            sim.step(1)
            if reached is None and sim.sim_truth()["wheel_speeds_mmps"][0] >= 0.9 * speed:
                reached = (tick + 1) * sim.dt
        x0 = sim.pose_mm()[0]
        sim.set_wheel_motors(0.0, 0.0)
        sim.advance(2.0)
        assert reached is not None, "the treads never reached 90% of the command"
        print(f"step to {speed:.0f} mm/s: 90% after {reached:.3f} s; command cut: rolls on {sim.pose_mm()[0] - x0:.1f} mm")


def edge_run(values: Dict[str, Any], level: str, speed: float) -> Dict[str, Any]:
    edge = 0.5 * values["table_length_mm"]
    sim = VectorSim(values=values, control_level=level, start_xy_mm=(edge - EDGE_START_MM, 0.0))
    result = sim.drive_straight(300.0, speed)
    sim.advance(1.5)
    assert sim.reactions in (0, 1), "at most one reaction in one drive"
    return {"result": result.result, "past_edge_mm": max(state.x_mm for state in sim.states) - edge,
            "fallen": sim.sim_truth()["fallen"], "reactions": sim.reactions}


def edges(v: Dict[str, Any]) -> None:
    for level in (CONTROL_DEFAULT, CONTROL_OVERRIDE):
        for speed in EDGE_SPEEDS:
            out = edge_run(v, level, speed)
            assert isinstance(out["fallen"], bool), "fallen must be a flag"
            print(f"edge, level {level}, {speed:.0f} mm/s: {out['result']}, front axle {out['past_edge_mm']:.1f} mm "
                  f"past the edge at most, fallen {out['fallen']}, reactions {out['reactions']}")


def stall_torque_sensitivity(doc: Dict[str, Any]) -> None:
    """What the two firmware comparisons do at other stall torques. Only the table's value is used anywhere else."""
    table = doc["parameters"]["wheel_stall_torque_mNm"]["value"]
    print(f"sensitivity to wheel_stall_torque_mNm (the table has {table:.0f}; the other values are not adopted):")
    for torque in STALL_TORQUES_MNM:
        values = params.values(doc, {"wheel_stall_torque_mNm": torque})
        rate = spin_rate(values, values["wheel_speed_max_mmps"])
        safe = [s for s in EDGE_SPEEDS if not edge_run(values, CONTROL_DEFAULT, s)["fallen"]]
        assert rate > 0.0, "the robot did not spin"
        print(f"  {torque:.0f} mN*m: spin at full speed {rate:.0f} deg/s; no fall with the reaction on at "
              f"{', '.join(f'{s:.0f}' for s in safe)} mm/s (of {', '.join(f'{s:.0f}' for s in EDGE_SPEEDS)})")


def rate() -> None:
    sim = VectorSim()
    sim.set_wheel_motors(60.0, 40.0)
    start = time.perf_counter()
    sim.advance(5.0)
    wall = time.perf_counter() - start
    assert wall > 0.0, "clock did not advance"
    print(f"5 s of robot time took {wall:.2f} s on one process ({5.0 / wall:.1f} times real time)")


def main() -> int:
    doc = params.load()
    v = params.values(doc)
    print("parameters:", params.summary(doc))
    drives()
    turns(v)
    spins(v)
    spin_fraction_against_the_recording(doc)
    scatter_in_motion(doc)
    step_and_coast(doc)
    edges(v)
    stall_torque_sensitivity(doc)
    rate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
