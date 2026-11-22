"""Checks of the rig-shaped body (simulacrum/body_mesh.py) against the real robot and against itself.

  python3 evidence/body-shape/validate.py BODY_DIR

1. Envelope at rest (lift down, head level): primitive model vs rig-shaped model vs the size retail
   listings give for Vector, 9.98 x 5.99 x 6.93 cm.
2. Drag: while the rig-shaped robot drives and turns on the flat table, no static piece (chassis,
   tread hulls, head, lift, fork) may touch the table; only the wheels may.
3. The fidelity measure can fail (fault-injection check): one collision piece of a part moved 2 mm, removed, or
   grown 20 %, must each raise the score past the build's limits; the unbroken part must pass.
4. Cost: wall time per simulated second, both models.
"""
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from simulacrum import body_mesh, mjcf, params  # noqa: E402
from simulacrum.robot import VectorSim  # noqa: E402

RETAIL_MM = (99.8, 59.9, 69.3)  # 9.98 x 5.99 x 6.93 cm, retail listings (no vendor spec sheet found)
FAULT_PART = "fork"


def envelope(sim, groups):
    """World AABB of the robot's geoms in the given geom groups, in mm (scene geoms excluded)."""
    m, d = sim.model, sim.data
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == 0 or int(m.geom_group[g]) not in groups or m.body_rootid[m.geom_bodyid[g]] != 1:
            continue
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            # MuJoCo re-centres a mesh on its principal axes; its bounding box there, turned back to
            # the world, overstates the extent. Use the mesh's own vertices.
            mesh = m.geom_dataid[g]
            local = m.mesh_vert[m.mesh_vertadr[mesh]: m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
        else:
            c, h = m.geom_aabb[g, :3], m.geom_aabb[g, 3:]
            local = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h + c
        world = local @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        lo, hi = np.minimum(lo, world.min(0)), np.maximum(hi, world.max(0))
    return lo / mjcf.MM, hi / mjcf.MM


def check_envelope(body):
    print("1. Envelope at rest, mm (length x width x height; retail 99.8 x 59.9 x 69.3)")
    for label, kw, groups in (("primitive (contact geoms)", {}, (1,)),
                              ("rig shape (visual meshes)", {"body": body}, (2,)),
                              ("rig shape (collision pieces)", {"body": body}, (1,))):
        sim = VectorSim(**kw)
        lo, hi = envelope(sim, groups)
        size = hi - lo
        delta = size - np.array(RETAIL_MM)
        print(f"   {label:30s} {size[0]:6.1f} x {size[1]:5.1f} x {size[2]:5.1f}   vs retail "
              f"{delta[0]:+5.1f} {delta[1]:+5.1f} {delta[2]:+5.1f}   x {lo[0]:.1f}..{hi[0]:.1f}")


def check_drag(body):
    print("2. Table contacts of static pieces while driving (must be 0)")
    sim = VectorSim(body=body)
    m = sim.model
    table = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "table")
    wheels = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"wheel_{w}_geom") for w in ("fl", "fr", "rl", "rr")}
    touched, ticks = {}, 0
    # Within 160 mm of the start: the table is far larger, so no edge and no cliff reaction.
    plan = [(150.0, 150.0, 1.0), (-150.0, 150.0, 1.0), (150.0, 150.0, 1.0), (0.0, 0.0, 0.5), (-150.0, -150.0, 1.0)]
    for left, right, seconds in plan:
        assert sim.set_wheel_motors(left, right), "robot refused the wheel command"
        for _ in range(int(seconds / sim.dt)):
            sim.step(1)
            ticks += 1
            for c in sim.data.contact[: sim.data.ncon]:
                pair = {int(c.geom1), int(c.geom2)}
                if table in pair and not pair & wheels:
                    other = (pair - {table}).pop()
                    touched[m.geom(other).name] = touched.get(m.geom(other).name, 0) + 1
    print(f"   {ticks} control ticks; static pieces that touched the table: {touched or 'none'}")
    return not touched


def rebuild_part(body, name):
    """The part's reference surface and its built pieces, read back from the build."""
    rig = Path(body["dir"]) / body_mesh.RIG_FILE
    verts_cm, groups = body_mesh.read_obj(rig)
    v_mm = body_mesh.to_robot(verts_cm, body["registration"])
    part = body_mesh.split_parts(v_mm, groups, body["frames"])[name]
    ref = body_mesh.merge(part["collide"])
    pieces = []
    for item in body["parts"][name]["collision"]:
        pv, pt = body_mesh.read_obj(Path(body["dir"]) / item["file"])
        pieces.append((pv, pt[0][1], item["friction"]))
    return ref, pieces


def check_faults(body):
    print(f"3. Fidelity measure under injected faults ({FAULT_PART}; limits {body_mesh.LIMITS})")
    ref, pieces = rebuild_part(body, FAULT_PART)
    big = int(np.argmax([len(p[0]) for p in pieces]))

    def moved(ps):
        v, t, f = ps[big]
        return ps[:big] + [(v + [2.0, 0.0, 0.0], t, f)] + ps[big + 1:]

    def grown(ps):
        v, t, f = ps[big]
        c = v.mean(0)
        return ps[:big] + [(c + 1.2 * (v - c), t, f)] + ps[big + 1:]

    arms = (("unbroken (control)", lambda ps: ps, True), ("largest piece moved 2 mm", moved, False),
            ("largest piece removed", lambda ps: ps[:big] + ps[big + 1:], False),
            ("largest piece grown 20 %", grown, False))
    caught = 0
    for label, fault, should_pass in arms:
        f = body_mesh.fidelity(*ref, fault(list(pieces)))
        passed = (f["protrusion_mm"]["p99"] <= body_mesh.LIMITS["protrusion_p99"]
                  and f["excess_mm"]["p95"] <= body_mesh.LIMITS["excess_p95"])
        verdict = "as expected" if passed == should_pass else "WRONG"
        caught += passed == should_pass
        print(f"   {label:26s} protrusion p99 {f['protrusion_mm']['p99']:5.2f}  excess p95 "
              f"{f['excess_mm']['p95']:5.2f}  {'pass' if passed else 'fail'} ({verdict})")
    print(f"   {caught} of {len(arms)} arms as expected")
    return caught == len(arms)


def check_cost(body):
    print("4. Wall time per simulated second (drive straight at 150 mm/s)")
    for label, kw in (("primitive", {}), ("rig shape", {"body": body})):
        sim = VectorSim(**kw)
        assert sim.set_wheel_motors(150.0, 150.0), "robot refused the wheel command"
        t0 = time.perf_counter()
        sim.advance(5.0)
        print(f"   {label:10s} {(time.perf_counter() - t0) / 5.0 * 1000.0:7.1f} ms per simulated s")


def main(body_dir):
    body = body_mesh.load(Path(body_dir))
    check_envelope(body)
    ok = check_drag(body)
    ok = check_faults(body) and ok
    check_cost(body)
    print("ALL CHECKS AS EXPECTED" if ok else "A CHECK FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    assert len(sys.argv) == 2, __doc__
    sys.exit(main(sys.argv[1]))
