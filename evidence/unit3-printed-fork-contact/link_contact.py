"""Compare fork-only and articulated-link replay without fitting the held-out pushes."""
import csv
import mujoco as mj
import numpy as np
import matplotlib.pyplot as plt
from evaluate import here,run,observed


def main():
    results=[]
    for trial in range(1,6):
        key=(trial,"stl","overhead","weld",0,0,0)
        for rails in (False,True):
            touches=[]
            checked=False
            def inspect(m,d,t,lift):
                nonlocal checked
                if trial==1 and rails and not checked and lift>65:
                    probe=mj.MjData(m)
                    probe.qpos[:]=d.qpos
                    probe.mocap_pos[:]=d.mocap_pos
                    probe.mocap_quat[:]=d.mocap_quat
                    probe.joint("cube_root").qpos[:3]=d.geom("rail1_13").xpos
                    mj.mj_forward(m,probe)
                    pairs=[[m.geom(int(i)).name for i in c.geom] for c in probe.contact]
                    assert any(any(n.startswith("rail1_") for n in pair) and
                               any(n.startswith("cube_") for n in pair) for pair in pairs)
                    checked=True
                for contact in d.contact:
                    names=[m.geom(int(i)).name for i in contact.geom]
                    if any(n.startswith("rail") for n in names) and any(n.startswith("cube") for n in names):
                        touches.append(t)
                        break
            score,trace=run(key,inspect if rails else None,rails=rails,observer_stride=1)
            if trial==1 and rails:
                assert checked
            results.append([trial,rails,score[-3],score[-1],trace[-1][2],len(touches),
                            min(touches,default=None),max(touches,default=None)])
    with (here/"link-contact.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow(["trial","links","endpoint_error_mm","peak_tilt_deg","left_travel_mm",
                    "link_contact_steps","first_contact_sec","last_contact_sec"])
        w.writerows(results)
    fig,ax=plt.subplots(figsize=(7,4))
    trials=np.arange(1,6)
    for on,off in ((False,-.20),(True,0)):
        ax.bar(trials+off,[r[4] for r in results if r[1]==on],.20,label="Fork + links" if on else "Fork only")
    ax.bar(trials+.20,observed["over_left"],.2,label="Physical")
    ax.set(xticks=trials,xlabel="Shelf push trial",ylabel="Leftward displacement (mm)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(here/"link-contact.png",dpi=150)


if __name__=="__main__":
    main()
