"""Does the model balance where simulacrum/wheelstand.py's closed form says?.

    ./run_in_ct.sh wheelstand_check.py        (needs MuJoCo; prints JSON, writes nothing)

For each stand, rear wheels and one tread edge:
- closed_form_deg: the balance angle from the whole robot's centre of mass as
  MuJoCo computes it (subtree centre of mass of the chassis, lift down, head
  level), through wheelstand.expected_deg;
- sim_deg: the angle found by bisection, releasing the model from rest at a
  tilt and seeing which way it falls, with the head and lift servos holding;
- other_contacts_at_balance: any geom touching the table at that angle other
  than the wheels it stands on (none means nothing else props it up);
- accel_read_deg: the tilt wheelstand.tilt_deg reads from the model's own
  accelerometer held still at that angle, with the table's offsets and noise.

One process, short runs: about 30 releases of 0.8 s of robot time each.
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Tuple

import mujoco

from simulacrum import mjcf, params, wheelstand
from simulacrum.robot import VectorSim

RELEASE_S = 0.8
BISECTIONS = 14
SEARCH = {"rear": (40.0, 85.0), "side": (30.0, 80.0)}
PIVOT_GEOMS = {"rear": {"wheel_rl_geom", "wheel_rr_geom"}, "side": {"wheel_fl_geom", "wheel_rl_geom"}}


def _quat_mul(a: Tuple[float, ...], b: Tuple[float, ...]) -> Tuple[float, float, float, float]:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def _tilt_quat(kind: str, deg: float) -> Tuple[float, float, float, float]:
    """Nose up by deg (rear) or left side down by deg (side)."""
    half = 0.5 * math.radians(deg)
    return (math.cos(half), 0.0, -math.sin(half), 0.0) if kind == "rear" else (math.cos(half), -math.sin(half), 0.0, 0.0)


def _pivot_mm(kind: str, v: Dict[str, Any]) -> Tuple[float, float, float]:
    """The point of the robot that stays on the table: under the rear axle, or the left tread's outer edge."""
    if kind == "rear":
        return (-v["wheel_base_mm"], 0.0, v["wheel_rear_radius_mm"])
    return (-0.5 * v["wheel_base_mm"], wheelstand.geometry(v)["edge_y"], 0.0)


def place(sim: VectorSim, rest_qpos: Any, kind: str, deg: float) -> None:
    """Put the settled robot at a tilt, pivot point on the table, nothing moving."""
    data, v = sim.data, sim.v
    data.qpos[:] = rest_qpos
    q = _tilt_quat(kind, deg)
    rest_q = tuple(float(c) for c in rest_qpos[3:7])
    data.qpos[3:7] = _quat_mul(rest_q, q)
    data.qpos[0:3] = (0.0, 0.0, 0.0)
    data.qvel[:] = 0.0
    mujoco.mj_forward(sim.model, data)
    pivot_world = _body_point(sim, _pivot_mm(kind, v))
    target_z = v["wheel_rear_radius_mm"] * mjcf.MM if kind == "rear" else 0.0
    data.qpos[2] += target_z - pivot_world[2]
    mujoco.mj_forward(sim.model, data)


def _body_point(sim: VectorSim, point_mm: Tuple[float, float, float]) -> List[float]:
    body = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    pos, mat = sim.data.xpos[body], sim.data.xmat[body].reshape(3, 3)
    local = [c * mjcf.MM for c in point_mm]
    return [float(pos[i] + sum(mat[i][j] * local[j] for j in range(3))) for i in range(3)]


def _tilt_now(sim: VectorSim, kind: str) -> float:
    body = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    mat = sim.data.xmat[body].reshape(3, 3)
    # World z components of the body's axes. atan2, not asin: a robot on its back is past 90 degrees.
    if kind == "rear":
        return math.degrees(math.atan2(float(mat[2][0]), float(mat[2][2])))
    return math.degrees(math.atan2(-float(mat[2][1]), float(mat[2][2])))


def falls_over(sim: VectorSim, rest_qpos: Any, kind: str, deg: float) -> bool:
    place(sim, rest_qpos, kind, deg)
    start = _tilt_now(sim, kind)
    steps = int(round(RELEASE_S / sim.model.opt.timestep))
    assert 0 < steps <= 20000, "release too long"
    mujoco.mj_step(sim.model, sim.data, steps)
    return _tilt_now(sim, kind) > start


def _other_contacts(sim: VectorSim, kind: str) -> List[str]:
    m, d = sim.model, sim.data
    name = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or str(int(g))
    out = set()
    for i in range(d.ncon):
        pair = {name(d.contact[i].geom1), name(d.contact[i].geom2)}
        robot = pair - {"table", "floor"}
        if robot - PIVOT_GEOMS[kind]:
            out.update(robot - PIVOT_GEOMS[kind])
    return sorted(out)


def whole_com_mm(sim: VectorSim) -> Tuple[float, float]:
    """The whole robot's centre of mass in the robot frame, settled, lift down and head level."""
    body = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    pos, mat = sim.data.xpos[body], sim.data.xmat[body].reshape(3, 3)
    rel = [float(sim.data.subtree_com[body][i] - pos[i]) for i in range(3)]
    local = [sum(mat[j][i] * rel[j] for j in range(3)) / mjcf.MM for i in range(3)]
    return local[0], local[2]


def stand(sim: VectorSim, rest_qpos: Any, kind: str, closed: float) -> Dict[str, Any]:
    low, high = SEARCH[kind]
    assert not falls_over(sim, rest_qpos, kind, low) and falls_over(sim, rest_qpos, kind, high), "no tipping point in range"
    for _ in range(BISECTIONS):
        mid = 0.5 * (low + high)
        if falls_over(sim, rest_qpos, kind, mid):
            high = mid
        else:
            low = mid
    found = 0.5 * (low + high)
    place(sim, rest_qpos, kind, found)
    # A hand holds the robot still: no acceleration, so the accelerometer reads gravity alone.
    # At the instant of placement the model is in free fall (no contact force yet), which is not that.
    sim.data.qacc[:] = 0.0
    mujoco.mj_rnePostConstraint(sim.model, sim.data)
    mujoco.mj_sensorAcc(sim.model, sim.data)
    sim._imu_sample = None
    sim._reset_imu_filter()  # held still: the chip's filter has settled on this reading
    sim._sample_imu()
    accel = sim._imu_sample[0]
    row = {"accel": list(accel), "head": float(sim.data.qpos[sim._q["head"]])}
    read = wheelstand.tilt_deg(row, sim.v)
    return {"closed_form_deg": closed, "sim_deg": found, "bracket_deg": [low, high], "placed_deg": _tilt_now(sim, kind),
            "other_contacts_at_balance": _other_contacts(sim, kind),
            "accel_read_deg": read[0] if kind == "rear" else read[1]}


def check(v: Dict[str, Any]) -> Dict[str, Any]:
    sim = VectorSim(values=v)
    rest_qpos = sim.data.qpos.copy()
    com_x, com_z = whole_com_mm(sim)
    closed = wheelstand.expected_deg(com_x, com_z, wheelstand.geometry(v))
    out: Dict[str, Any] = {"model_com_mm": [com_x, com_z], "table_com_mm": [v["com_x_mm"], v["com_z_mm"]]}
    for kind in wheelstand.KINDS:
        out[kind] = stand(sim, rest_qpos, kind, closed[kind])
    return out


def main() -> int:
    result = check(params.values(params.load()))
    print(json.dumps(result, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
