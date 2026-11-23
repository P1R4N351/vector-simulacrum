"""Replay interpolated reported speeds as shaft velocities and score the optical interval."""
import csv
import io
import json
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from recorded_mount import ROOT, read_trial, table


def replay(trials, case):
    export = """import {rearEnvelope} from './rear_envelope.mjs'; import {readText} from '../../web/test/helpers.js';
    process.stdout.write(rearEnvelope(readText('public/robot/vector2/vector-free-dock.xml'),'driven_rear_envelope'));"""
    xml = subprocess.check_output(["node","--input-type=module","-e",export],cwd=ROOT,text=True)

    model = ET.fromstring(xml)
    shaft = case.startswith("Shaft")
    if shaft:
        for name in ["left_wheel","right_wheel"]:
            motor=model.find(f".//actuator/general[@name='{name}']")
            cap=float(motor.get("gainprm").split()[0])
            joint=motor.get("joint")
            motor.clear()
            motor.tag="velocity"
            motor.attrib.update(name=name,joint=joint,kv=case.split()[1],forcelimited="true",forcerange=f"{-cap} {cap}")
    with tempfile.TemporaryDirectory() as tmp:
        path=Path(tmp)/"model.xml"
        ET.ElementTree(model).write(path)
        inputs=[dict(trial, shaft_replay=shaft, interpolate=case!="Held command", immediate_wheels=case=="Immediate linear") for trial in trials]
        raw=subprocess.check_output(["node",str(ROOT/"recorded_mount.mjs"),str(path)],input=json.dumps(inputs),
                                    env=dict(os.environ,VECTOR_TRACE_SHAFTS="1"),text=True)
    data=np.genfromtxt(io.StringIO(raw),names=True,delimiter=",",dtype=None,encoding="utf8")
    return data


def main():
    trials = [read_trial(c, o)[0] for c, o in zip(table("return-scores.csv"), table("mount-target-inputs.csv"), strict=True)]
    optical = table("fork-yaw-motion.csv")
    scores = []
    fig, axes = plt.subplots(2,5,figsize=(14,6),sharex="col",sharey="row",layout="constrained")
    for case in ["Held command","Linear command","Shaft 0.08","Shaft 0.8"]:
        data=replay(trials, case)
        for col,trial in enumerate(trials):
            name=trial["name"]
            run=data[data["session"]==name]
            cam=optical[optical["session"]==name]
            t=run["t_s"]
            src,left,right=np.array(trial["wheels"]).T
            before=src<0
            wl=np.interp(src[before],t,run["shaft_l"])
            wr=np.interp(src[before],t,run["shaft_r"])
            mae=np.mean(np.abs(np.c_[wl-left[before],wr-right[before]]))
            yaw=run["yaw_deg"]-np.interp(cam["t_s"][0],t,run["yaw_deg"])
            on=t[run["on"]==1]
            scores.append([name,case,mae,np.interp(cam["t_s"][-1],t,yaw),cam["tip_deg"][-1],
                           cam["gyro_deg"][-1],on[0] if len(on) else ""])
            a,b=axes[:,col]
            if case=="Held command":
                a.plot(src,right-left,color="black",lw=1.3,label="Measured")
                b.plot(cam["t_s"],cam["tip_deg"],"ko-",label="Fork tips")
            a.plot(t,run["shaft_r"]-run["shaft_l"],lw=1,label=case)
            b.plot(t,yaw,lw=1,label=case)
            a.set_title(f"Return {name}")
            b.set(xlabel="From physical on-report (s)",xlim=(-1,0))
    axes[0,0].set_ylabel("Right − left speed (mm/s)")
    axes[1,0].set_ylabel("Yaw from first camera sample (°)")
    for ax in axes[:,0]:
        ax.legend(fontsize=7)
    fig.savefig(ROOT/"mount-shaft-replay.png",dpi=160)
    with (ROOT/"mount-shaft-replay.csv").open("w") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow("session replay shaft_mae_mmps optical_window_sim_deg camera_tip_deg gyro_deg first_contact_s".split())
        writer.writerows(scores)


if __name__=="__main__":
    main()
