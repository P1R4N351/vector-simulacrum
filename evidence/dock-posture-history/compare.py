"""Score at physical packet times and show the two independent hub measurements."""
import csv
import json
import sys

import matplotlib.pyplot as plt
import numpy as np

physical = np.asarray(json.load(sys.stdin))
camera = np.genfromtxt("../unit3-w34-return/departure-hubs.csv", delimiter=",", names=True)
scale = 48 / (camera["rear_x"][0] - camera["front_x"][0])
time = camera["t_s"]
fig, ((head, wheels), (travel, pose)) = plt.subplots(2,2,figsize=(11,7), layout="constrained")
head.plot(physical[:,0], physical[:,3], color="black", lw=1.2, label="Physical head")
speed = physical[:,1:3].mean(axis=1)
wheels.step(physical[:,0], speed, where="post", color="black",label="Physical wheel mean")
for hub in ["front_x","rear_x"]:
    displacement = (camera[hub][0]-camera[hub]+camera["static_x"]-camera["static_x"][0])*scale
    travel.plot(time,displacement,".-",label=hub.split("_")[0]+" hub")
    print(hub,"at last-on and first-off:",np.interp([-.081,0],time,displacement))
modes = ["static","posture","wheels","full","noslip","shaft","shaft-noslip","shaft-linear"]
with open("scores.csv","w") as output:
    writer=csv.writer(output,lineterminator="\n")
    writer.writerow(["mode","first_off_ms","dwell_travel_to_camera_start_mm","at_last_on_mm",
                     "at_first_off_mm","at_sim_release_mm","head_rmse_deg","lift_rmse_mm"])
    for mode in modes:
        r=np.genfromtxt(mode+".csv", delimiter=",",names=True)
        t=r["s"]
        start=np.interp(time[0],t,r["x_mm"])
        x=r["x_mm"]-start
        off=np.flatnonzero(r["charger"]==0)[0]
        err=lambda col,i: np.sqrt(np.mean((np.interp(physical[:,0],t,r[col])-physical[:,i])**2))
        writer.writerow([mode,1000*t[off],start-r["x_mm"][0],np.interp(-.081,t,x),
                         np.interp(0,t,x),x[off],err("head_deg",3),err("lift_mm",4)])
        if mode in ["full","shaft","shaft-noslip"]:
            travel.plot(t,x,label=mode)
            travel.scatter(t[off],x[off],marker="x")
            pose.plot(t,r["x_mm"]-r["x_mm"][0], label=mode)
        if mode in ["full","shaft"]:
            wheels.plot(t,(r["wl"]+r["wr"])/2, label=mode)
        if mode=="full":
            head.plot(t,r["head_deg"],lw=1,label="Full history replay")
head.set(xlabel="Seconds from first off",ylabel="Head angle (°)",title="Recorded dock posture")
wheels.set(xlim=(-.5,.5),xlabel="Seconds from first off",ylabel="Wheel speed (mm/s)",title="Departure wheel tracking")
travel.axvspan(-.081,0,color="grey",alpha=.18)
travel.set(xlim=(-.48,.53),ylim=(-1,19),xlabel="Seconds from first off",ylabel="Travel from first camera frame (mm)",title="Hub tracks and simulated travel (× = release)")
pose.set(xlabel="Seconds from first off",ylabel="World X change (mm)",title="Simulated travel since first dock contact")
for ax in [head,wheels,travel,pose]:
    ax.legend(fontsize=8)
fig.savefig("history.png",dpi=155)
