"""Test only the part of the baseline chassis box inside the fixed rig shell's convex envelope."""
import csv
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection

import replay
from chassis_patch import shell_triangles

HERE = Path(__file__).resolve().parent


def clip_box(tree, triangles):
    geom = tree.find(".//geom[@name='chassis_geom']")
    center = np.fromstring(geom.attrib.pop("pos"),sep=" ")*1000
    half = np.fromstring(geom.attrib.pop("size"),sep=" ")*1000
    planes = ConvexHull(np.unique(triangles.reshape(-1,3),axis=0)).equations
    box = np.vstack((np.column_stack((np.eye(3),-center-half)),np.column_stack((-np.eye(3),center-half))))
    planes = np.vstack((planes,box))
    assert np.max(planes[:,:3]@center+planes[:,3]) < 0
    vertices = HalfspaceIntersection(planes,center).intersections
    ET.SubElement(tree.find("asset"),"mesh",name="clipped_chassis",vertex=" ".join(f"{x:.10g}" for x in vertices.ravel()*.001))
    geom.set("type","mesh")
    geom.set("mesh","clipped_chassis")


def main(fixture,rig):
    v = replay.params.values(replay.params.load(model="vector_2"), {"tip_supports_driven":1})
    tris = shell_triangles(Path(rig),v)
    models = {"Box":replay.build(v,fixture),"Clipped box":replay.build(v,fixture,lambda xml:clip_box(xml,tris))}
    fig,axs = plt.subplots(2,2,figsize=(11,6),layout="constrained",sharex="col")
    with (HERE/"clipped-chassis.csv").open("w",newline="") as f:
        w = csv.writer(f,lineterminator="\n")
        w.writerow(["trial","case","proxy","late_taps","peak_pitch_deg","hold_pitch_deg"])
        for col,name in enumerate(("W4","W8")):
            tr = replay.Trial(name)
            p = tr.cube[::3]
            t = p["sec"]-tr.zero
            lo,hi = tr.late-.3,min(t[-1],tr.late+1.2)
            physical = t[1:][np.diff(p["tap"])!=0]
            last = max(physical[(physical>=lo)&(physical<=hi)])
            ax, taps = axs[:,col]
            ax.plot(tr.state["sec"]-tr.zero,np.rad2deg(tr.state["pitch"]),"k.-",label="Recorded")
            taps.scatter(physical,np.zeros_like(physical),marker="|",s=120,color="black",label="Recorded")
            for k,(label,m) in enumerate(models.items()):
                m.opt.timestep = v["sim_timestep_ms"]*.001/4
                for case,(s,o) in enumerate(((s,o) for s in (3.5,3.8) for o in (tr.early,tr.late)),1):
                    events = []
                    trace,pk,n,_ = replay.run(m,v,tr,s+tr.shift,o,"servo:60",17,tap_times=events)
                    metrics = replay.score(tr,trace,pk,n,o)
                    events = np.array(events)
                    w.writerow([name,case,label,sum((events>last+.06)&(events<=hi)),metrics["peak_pitch_deg"],metrics["hold_pitch_deg"]])
                    if case==1:
                        ax.plot(trace[:,0],trace[:,1],color=f"C{k}",label=label)
                    taps.scatter(events,np.full(events.size,case+(k-.5)*.25),s=60,marker="|",color=f"C{k}",label=label if case==1 else None)
            ax.set_title(name+" · pitch for timing case 1")
            taps.axvline(last+.06,ls=":",color="0.4",label="Late cutoff")
            taps.set_yticks(range(5),["Recorded","1","2","3","4"])
            taps.set_xlabel("Seconds after wheelstand request")
            taps.set_xlim(lo,hi)
    axs[0,0].set_ylabel("Chassis pitch (°)")
    axs[1,0].set_ylabel("Tap packet by timing case")
    axs[0,0].legend()
    axs[1,0].legend(fontsize=7,loc="upper right")
    fig.savefig(HERE/"clipped-chassis.png",dpi=160)


if __name__ == "__main__":
    main(*sys.argv[1:])
