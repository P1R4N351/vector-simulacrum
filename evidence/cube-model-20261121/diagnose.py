"""What the model's lift can hold, and what a spin with the cube in the lift shows. Authored-by: CPCS (cpcs-66ffc23).

    ./run_in_ct.sh evidence/cube-model-20261121/diagnose.py

Part 1: for cube masses from 10 to 60 g, the lift height 0.2 s and 1.2 s after the settle, with every other value
from the table. Part 2: with a cube the lift holds, a still second, then a turn in place at +/-80 mm/s; it
prints the turn centre the cube's messages give, where the chip really circles, the chassis tilt and the robot
IMU's own reading, still and turning. Prints one JSON object per line.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import mujoco  # noqa: E402

from simulacrum import cube, params  # noqa: E402
from simulacrum.robot import VectorSim  # noqa: E402

DOC = params.load()
WHEEL_MMPS, STILL_S, SPIN_UP_S, MEASURE_S = 80.0, 1.0, 2.0, 2.0


def tilt_deg(sim: VectorSim) -> list:
    """Roll and pitch of the chassis, in degrees, from its rotation matrix."""
    body = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    r = np.array(sim.data.xmat[body]).reshape(3, 3)
    return [round(math.degrees(math.atan2(r[2, 1], r[2, 2])), 3), round(math.degrees(-math.asin(r[2, 0])), 3)]


def holds() -> None:
    for mass in (10.0, 20.0, 25.0, 30.0, 40.0, 50.0, 60.0):
        sim = VectorSim(values=params.values(DOC, {"cube_mass_g": mass}), cube="lift", noise_seed=1)
        at0 = sim.lift_height_mm()
        sim.advance(0.2)
        at02 = sim.lift_height_mm()
        sim.advance(1.0)
        print(json.dumps({"part": "hold", "cube_mass_g": mass, "lift_mm_at_0": round(at0, 2),
                          "lift_mm_at_0.2s": round(at02, 2), "lift_mm_at_1.2s": round(sim.lift_height_mm(), 2)}))


def window(sim: VectorSim, seconds: float, site: int) -> dict:
    path, frames, yaws, times, tilts, imu = [], [], [], [], [], []
    for _ in range(int(round(seconds / sim.dt))):
        sim.step(1)
        path.append(np.array(sim.data.site_xpos[site][:2]))
        frames.append(np.array(sim.data.site_xmat[site]).reshape(3, 3))
        yaws.append(sim.heading_total_rad())
        times.append(float(sim.data.time))
        tilts.append(tilt_deg(sim))
        imu.append(sim._imu()[0])
    return {"path": np.array(path), "frames": frames, "omega": (yaws[-1] - yaws[0]) / (times[-1] - times[0]),
            "tilt": np.mean(tilts, axis=0).round(3).tolist(), "imu_mmps2": np.mean(imu, axis=0).round(1).tolist()}


def spin(mass: float) -> None:
    sim = VectorSim(values=params.values(DOC, {"cube_mass_g": mass}), cube="lift", noise_seed=6)
    v = sim.v
    site = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_SITE, "cube_chip")
    still = window(sim, STILL_S, site)
    n_still = len(sim.cube.messages)
    sim.set_wheel_motors(-WHEEL_MMPS, WHEEL_MMPS)
    sim.advance(SPIN_UP_S)
    first = len(sim.cube.messages)
    turning = window(sim, MEASURE_S, site)
    got = cube.turn_centre(cube.samples_g(cube.rows(sim.cube.messages[:n_still]), v["cube_accel_counts_per_g"]),
                           cube.samples_g(cube.rows(sim.cube.messages[first:]), v["cube_accel_counts_per_g"]),
                           turning["omega"], v["gravity_mmps2"])
    pts = turning["path"]
    a = np.column_stack([2.0 * pts, np.ones(len(pts))])
    cx, cy, c = np.linalg.lstsq(a, (pts ** 2).sum(axis=1), rcond=None)[0]
    radius = math.sqrt(c + cx * cx + cy * cy) * 1000.0
    truth = np.mean([(f.T @ np.array([cx - p[0], cy - p[1], 0.0]))[:2] for f, p in zip(turning["frames"], pts)], axis=0)
    # What a tilt does: gravity's share in x and y, in mm of apparent centre at this turn rate.
    print(json.dumps({"part": "spin", "cube_mass_g": mass, "omega_radps": round(turning["omega"], 4),
                      "cube_centre_mm": [round(x, 2) for x in got["centre_mm"]],
                      "chip_circles_centre_mm": (truth * 1000.0).round(2).tolist(), "chip_circle_radius_mm": round(radius, 2),
                      "cube_delta_g": [round(x, 5) for x in got["delta_g"]],
                      "tilt_still_deg": still["tilt"], "tilt_turning_deg": turning["tilt"],
                      "robot_imu_still_mmps2": still["imu_mmps2"], "robot_imu_turning_mmps2": turning["imu_mmps2"],
                      "lift_mm_end": round(sim.lift_height_mm(), 2), "samples": got["samples"]}))


def main() -> int:
    holds()
    for mass in (20.0, 25.0):
        spin(mass)
    return 0


if __name__ == "__main__":
    sys.exit(main())
