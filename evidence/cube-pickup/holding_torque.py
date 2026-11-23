"""Torque the lift motor must hold, with and without a carried cube, across the lift's travel.

    python evidence/cube-pickup/holding_torque.py

For each lift height the lift joint is set there, the fork level, at rest. The fork is kept level by a
joint equality (wrist = -lift, as the real parallel linkage keeps it), so a turn of the lift joint by
d turns the wrist by -d, and the motor must supply the gravity torque on the lift joint less that on
the wrist joint: qfrc_bias[lift] - qfrc_bias[wrist] at zero velocity, which is gravity alone. The
carried cube is the model's own cube="lift" mount (cube_carry_offset_mm, cube_mass_g), weighed at the
mass table's low, value and high. The answer is a lower bound on the lift motor's torque limit: below
it the lift cannot hold a cube up, let alone raise one.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from simulacrum import mjcf, params  # noqa: E402

HEIGHTS_MM = tuple(range(32, 93, 5)) + (92,)


def holding(v, cube) -> list:
    model = mujoco.MjModel.from_xml_string(mjcf.build_mjcf(v, (), cube))
    data = mujoco.MjData(model)
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift_joint")
    wid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "wrist_joint")
    assert jid >= 0 and wid >= 0, "the model has no lift or wrist joint"
    lift_dof, wrist_dof = model.jnt_dofadr[jid], model.jnt_dofadr[wid]
    out = []
    for h in HEIGHTS_MM:
        mujoco.mj_resetData(model, data)
        angle = mjcf.lift_angle_of_height(v, float(h))
        data.qpos[model.jnt_qposadr[jid]] = angle
        data.qpos[model.jnt_qposadr[wid]] = -angle  # the fork stays level, as the linkage holds it
        lower = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift_lower_joint")
        if lower >= 0:
            data.qpos[model.jnt_qposadr[lower]] = angle
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        torque = float(data.qfrc_bias[lift_dof] - data.qfrc_bias[wrist_dof])
        out.append([h, round(abs(torque) * 1000.0, 2)])
    return out


def main() -> int:
    table = params.load()
    v = params.values(table)
    mass = table["parameters"]["cube_mass_g"]
    rows = {"no cube": holding(v, None)}
    for label, grams in (("low", mass["low"]), ("value", mass["value"]), ("high", mass["high"])):
        rows[f"cube {grams:g} g ({label})"] = holding(dict(v, cube_mass_g=float(grams)), "lift")
    for name, series in rows.items():
        worst = max(series, key=lambda r: r[1])
        print(f"{name:>22}: worst {worst[1]:6.2f} mN*m at {worst[0]} mm; "
              + " ".join(f"{h}:{t:.1f}" for h, t in series))
    print(json.dumps({"lift_servo_torque_max_mNm": v["lift_servo_torque_max_mNm"], "rows": rows}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
