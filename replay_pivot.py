"""Track instantaneous pivot and ground contact while replaying nine Vector 2.0 turns."""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from replay_measured_turn import run

ROOT = Path(__file__).resolve().parent
DATA = ROOT / "evidence/endurance-drift"


def configure_round(root):
    # Replace only the four 1 mg pressure proxies. The full-width wheel
    # cylinders retain axle inertia and serve as the existing tip supports.
    for geom in root.findall(".//geom"):
        if geom.get("name", "").startswith("wheel_patch_"):
            geom.set("type", "sphere")
            del geom.attrib["fromto"]


def configure_plane(root):
    # Diagnostic interior-only surface. No table edge exists in this model.
    geom = root.find(".//geom[@name='table']")
    geom.attrib.update(type="plane", pos="0 0 0", size="0 0 1")


CONFIGURATIONS = {"baseline": None, "round": configure_round, "plane": configure_plane}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", choices=["balanced","measured"])
    parser.add_argument("contact", choices=CONFIGURATIONS)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    events = list(map(json.loads,(DATA/"run-g.jsonl").read_text().splitlines()))
    rows = [r for r in events if r["k"]=="s" and 44.90<=r["t"]<=92.1]
    t = np.array([r["t"] for r in rows])
    wheels = np.array([[r["wl"],r["wr"]] for r in rows])
    lo,hi = np.argmax(np.diff(t)), np.searchsorted(t,91.49)
    t[lo:hi+1] = np.linspace(t[lo],t[hi],hi-lo+1)
    if args.input=="balanced":
        half = np.diff(wheels,axis=1)/2
        wheels = np.column_stack((-half,half))

    names = {}
    samples = []
    tick = 0
    def contacts(m,d,now):
        nonlocal tick
        tick += 1
        if tick % 5 != 1:
            return
        if not names:
            names.update({i:m.geom(i).name for i in range(m.ngeom)})
        force = np.zeros(6)
        loads = np.zeros(4)
        moment = np.zeros(4)
        support = 0.
        for j,c in enumerate(d.contact):
            pair = [names[c.geom1], names[c.geom2]]
            if "table" not in pair:
                continue
            name = pair[1-pair.index("table")]
            mujoco.mj_contactForce(m,d,j,force)
            if name.startswith("wheel_patch_"):
                k = ("fl","fr","rl","rr").index(name[-2:])
                loads[k] += force[0]
                offset = c.pos - d.geom_xpos[m.geom(name).id]
                # lateral offset relative to each patch's own centerline
                lateral = d.xmat[m.body("chassis").id].reshape(3,3)[:,1]
                moment[k] += force[0]*np.dot(offset,lateral)*1000
            else:
                support += force[0]
        samples.append([now,*loads,*moment,support])
    clock,out = run(rows,t,wheels,42,dense=True,include_pose=True,
                    configure=CONFIGURATIONS[args.contact],observe=contacts)
    np.savez_compressed(args.output,t=clock,out=out,contact=np.asarray(samples))


if __name__ == "__main__":
    main()
