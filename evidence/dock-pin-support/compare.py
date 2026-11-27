"""Compare the observed departure bracket with the replayed electrical and mechanical edges."""
import csv
import matplotlib.pyplot as plt
import numpy as np


def read(path):
    return np.genfromtxt(path, names=True, delimiter=",")


fig, (rest, motion, edge) = plt.subplots(1, 3, figsize=(13, 4.2), layout="constrained")
hub = read("../unit3-w34-return/departure-hubs.csv")
scale = 48 / (hub["rear_x"][0] - hub["front_x"][0])
for k in ("front_x", "rear_x"):
    travel = (hub[k][0] - hub[k] + hub["static_x"] - hub["static_x"][0]) * scale
    motion.plot(hub["t_s"], travel, ".-", label=k.split("_")[0]+" hub")
motion.axvspan(-0.081, 0, color="grey", alpha=0.18)
edge.fill_between([-81,0], 6.6, 10.7, color="grey", alpha=0.35, label="Physical bracket")
with open("scores.csv", "w") as f:
    writer = csv.writer(f, lineterminator="\n")
    writer.writerow(["solver","dwell_s","passive_travel_mm","first_off_ms","release_travel_mm","last_loaded_ms","last_loaded_travel_mm"])
    for i, solver in enumerate([0,50,150,300,"noslip"]):
        color = "C"+str(i)
        if solver != "noslip":
            r = read(f"spring-{solver}.csv")
            rest.plot(r["time_s"], r["x_mm"]-r["x_mm"][0], color=color, label=f"{solver} N/m" if solver else "Canonical")
        for dwell in (0,57.8):
            name = (f"noslip-{dwell}.csv" if solver=="noslip" else
                    f"{'dwell' if dwell else 'depart'}-{solver}.csv")
            r = read(name)
            off = np.flatnonzero(r["on"]==0)[0]
            loaded = np.flatnonzero((r["left_pin_n"]>0)&(r["right_pin_n"]>0))
            t, x = r["t_s"][off]*1000, r["travel_mm"][off]
            writer.writerow([solver,dwell,r["dwell_travel_mm"][0],t,x,
                             r["t_s"][loaded[-1]]*1000 if loaded.size else "",
                             r["travel_mm"][loaded[-1]] if loaded.size else ""])
            edge.scatter(t, x, marker="x" if dwell else "o", color=color)
            if solver==0:
                motion.plot(r["t_s"],r["travel_mm"],label="Canonical, "+str(dwell)+" s dwell")
rest.set(xlabel="Simulation time (s)",ylabel="Downhill travel (mm)",title="Resting on dock")
motion.set(xlim=(-0.48,0.53),ylim=(-1,19),xlabel="Seconds from physical first off",ylabel="Travel (mm)",title="W35 camera and canonical replay")
edge.set(xlabel="Release relative to physical first off (ms)",ylabel="Travel to release (mm)",title="Circle: fresh seat    ×: after dwell")
for ax in (rest,motion,edge):
    ax.legend(fontsize=8)
fig.savefig("pin-support.png",dpi=160)
