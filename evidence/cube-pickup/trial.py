"""Can the simulated robot pick up a free light cube with its lift?

    python evidence/cube-pickup/trial.py [--json] [--set=name=value ...]

The robot starts square to the cube's -X face with its lift at the bottom (the docking height),
drives forward until its fork meets the cube and PUSH_MM more, raises the lift to the carry height,
holds it there, then backs away BACK_MM. A pickup counts when the cube ends at least PICKUP_RISE_MM
above where it started and is still that high after the robot has backed away. It is run at several
lateral offsets, since a real dock lands within a millimetre or two of centre. While the cube is held,
its centre is also given in the fork's frame, beside the engine's own carry offset
(cube_carry_offset_mm), which the engine uses to place a carried cube.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from simulacrum import mjcf, params  # noqa: E402
from simulacrum.robot import VectorSim  # noqa: E402

GAP_MM = 15.0            # fork front to cube face at the start
PUSH_MM = 7.0            # driven past first contact, as docking does
SPEED_MMPS = 20.0
HOLD_S = 1.5
BACK_MM = 40.0
PICKUP_RISE_MM = 40.0    # of the lift's 60 mm travel
OFFSETS_MM = (-2.0, -1.0, 0.0, 1.0, 2.0)


def _body(sim: VectorSim, name: str) -> int:
    bid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, name)
    assert bid >= 0, f"the model has no {name}"
    return bid


def cube_pose(sim: VectorSim):
    """The cube's centre (world mm) and its tilt from upright (degrees)."""
    bid = _body(sim, "cube")
    pos = np.array(sim.data.xpos[bid]) / mjcf.MM
    zaxis = np.array(sim.data.xmat[bid]).reshape(3, 3)[:, 2]
    return pos, math.degrees(math.acos(max(-1.0, min(1.0, float(zaxis[2])))))


def cube_in_fork_mm(sim: VectorSim):
    """The cube's centre in the fork's frame (mm): x forward, z up, origin at the wrist."""
    fork, cube = _body(sim, "lift_fork"), _body(sim, "cube")
    rot = np.array(sim.data.xmat[fork]).reshape(3, 3)
    rel = rot.T.dot(np.array(sim.data.xpos[cube]) - np.array(sim.data.xpos[fork])) / mjcf.MM
    return [round(float(rel[0]), 1), round(float(rel[1]), 1), round(float(rel[2]), 1)]


def fork_front_x_mm(sim: VectorSim) -> float:
    """World x of the frontmost point of any contact geom on the fork body."""
    m, d = sim.model, sim.data
    fork = _body(sim, "lift_fork")
    front = -1e9
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != fork or m.geom_contype[g] == 0:
            continue
        rot = np.array(d.geom_xmat[g]).reshape(3, 3)
        front = max(front, float(d.geom_xpos[g][0] + np.abs(rot[0]).dot(np.array(m.geom_size[g]))))
    assert front > -1e9, "the fork has no contact geoms"
    return front / mjcf.MM


def trial(v, offset_mm: float) -> dict:
    """One dock, lift, hold and back-away at a lateral offset (mm) from the cube's centre line."""
    probe = VectorSim(v, cube="free", lift_mm=v["lift_height_min_mm"])
    face = cube_pose(probe)[0][0] - 0.5 * v["cube_outer_mm"]
    start_x = face - GAP_MM - (fork_front_x_mm(probe) - probe.pose_mm()[0])
    sim = VectorSim(v, cube="free", start_xy_mm=(start_x, offset_mm), lift_mm=v["lift_height_min_mm"])
    rest, _ = cube_pose(sim)
    drive = sim.drive_straight(GAP_MM + PUSH_MM, SPEED_MMPS)
    sim.advance(0.3)
    docked, _ = cube_pose(sim)
    lift = sim.set_lift_height(v["lift_height_max_mm"])
    sim.advance(HOLD_S)
    held, held_tilt = cube_pose(sim)
    in_fork = cube_in_fork_mm(sim)
    sim.drive_straight(-BACK_MM, SPEED_MMPS)
    sim.advance(0.5)
    carried, carried_tilt = cube_pose(sim)
    rise, still = float(held[2] - rest[2]), float(carried[2] - rest[2])
    return {
        "offset_mm": offset_mm, "drive_result": drive.result, "lift_result": lift.result,
        "lift_height_mm": round(sim.lift_height_mm(), 1),
        "cube_pushed_mm": round(float(docked[0] - rest[0]), 1),
        "cube_rise_mm": round(rise, 1), "cube_tilt_deg": round(held_tilt, 1),
        "cube_in_fork_mm": in_fork,
        "rise_after_backing_mm": round(still, 1), "tilt_after_backing_deg": round(carried_tilt, 1),
        "picked_up": rise >= PICKUP_RISE_MM and still >= PICKUP_RISE_MM,
    }


def main(argv) -> int:
    v = params.values(params.load())
    for arg in argv[1:]:
        if arg.startswith("--set="):
            name, value = arg[len("--set="):].split("=", 1)
            assert name in v, f"no parameter {name}"
            v[name] = json.loads(value)
            print(f"override {name} = {v[name]}")
    rows = [trial(v, off) for off in OFFSETS_MM]
    if "--json" in argv:
        print(json.dumps({"carry_offset_mm": v["cube_carry_offset_mm"], "rows": rows}, indent=1))
    else:
        for r in rows:
            print(" ".join(f"{k}={r[k]}" for k in r))
        print("engine carry offset (cube centre from the wrist, fork frame):", v["cube_carry_offset_mm"])
    print("picked up %d of %d" % (sum(r["picked_up"] for r in rows), len(rows)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
