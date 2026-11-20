"""Compare baseline and round pressure patches on endurance and independent turns."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from replay_endurance_drift import relative_center
from replay_native_stops import DATA as NATIVE, LIFT_MM, TEST, delayed
from replay_pivot import DATA, ROOT, CONFIGURATIONS
from replay_measured_turn import run

FIG = ROOT / "evidence/physics-plots"
COLORS = {"baseline":"#ab513f", "round":"#16816f","plane":"#7059a5"}


def native_test(ax):
    rows = list(csv.DictReader((NATIVE/"unit2-native-replay-input.csv").open()))
    ax.set_title("Independent 100 mm/s stops · both directions")
    for turn in TEST:
        group = [r for r in rows if r["turn"]==turn]
        def col(k):
            return np.array([float(r[k]) for r in group])
        t = col("ms_from_last_edge")/1000
        wheels = np.column_stack((col("left_mmps"),col("right_mmps")))
        measured = np.degrees(col("head_gz"))
        ax.plot(t,measured,color="#232f40",lw=2,label="Hardware" if turn==TEST[0] else None)
        for name in COLORS:
            c,out = run([{"head":col("head_rad")[0],"lift":LIFT_MM}],t,wheels,42,dense=True,
                        configure=CONFIGURATIONS[name])
            pred = delayed(c,out[:,0],t,17)
            ax.plot(t,pred,color=COLORS[name],ls="--" if name=="baseline" else ":",
                    label=name if turn==TEST[0] else None)
            for low,high in ((-.85,-.25),(-.10,.10)):
                mask = (t>=low)&(t<=high)
                print("native",turn,name,low,round(np.mean((pred[mask]-measured[mask])**2)**.5,2))
    ax.set(xlim=(-.9,.12),xlabel="Seconds from last encoder edge",ylabel="Head gyro Z (°/s)")
    ax.legend(fontsize=8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cache",type=Path,help="Directory with pivot-{input}-{contact}.npz")
    args = parser.parse_args()
    events = list(map(json.loads,(DATA/"run-g.jsonl").read_text().splitlines()))
    starts = [r["t"] for r in events if r.get("name")=="wheels"]
    stops = [r["t"] for r in events if r.get("why")=="segment_end"]
    rows = [r for r in events if r["k"]=="s" and 44.90<=r["t"]<=92.1]
    t = np.array([r["t"] for r in rows])
    g = np.loadtxt(DATA/"head-gyro.csv",delimiter=",",skiprows=1)
    assert np.array_equal(g[:,0],t)
    gyro = np.degrees(g[:,1])
    center = relative_center(np.array([r["pose"][:2] for r in rows]),np.array([r["yaw"] for r in rows]))
    ends = [0,*[np.searchsorted(t,v)-1 for v in starts[1:]],len(t)-1]
    pivot,axes = plt.subplots(3,3,figsize=(12,8),sharex=True,sharey=True,layout="constrained")
    fig,(a,b,native) = plt.subplots(3,1,figsize=(10,10),layout="constrained")
    a.plot(range(1,10),np.linalg.norm(np.diff(center[ends],axis=0),axis=1),"o-",color="#232f40",label="SDK odometry")
    table = []
    traces = []
    for mode in ("balanced","measured"):
        for name,color in COLORS.items():
            data = np.load(args.cache/f"pivot-{mode}-{name}.npz")
            clock,out,contact = data["t"],data["out"],data["contact"]
            yaw = np.unwrap(out[:,5])
            xy = relative_center(out[:,3:5],yaw)
            rest = np.searchsorted(clock,t[ends]).clip(max=len(clock)-1)
            travel = np.linalg.norm(np.diff(xy[rest],axis=0),axis=1)
            # Central difference over 20 ms; reject near-rest divisions.
            dt = clock[20:]-clock[:-20]
            vel = (xy[20:]-xy[:-20])/dt[:,None]
            omega = (yaw[20:]-yaw[:-20])/dt
            spin = np.abs(omega)>np.pi/2
            offset = np.full_like(vel,np.nan)
            offset[spin] = np.column_stack((-vel[spin,1],vel[spin,0]))/omega[spin,None]
            angle = yaw[10:-10]-yaw[0]
            body = np.column_stack((offset[:,0]*np.cos(angle)+offset[:,1]*np.sin(angle),
                                    -offset[:,0]*np.sin(angle)+offset[:,1]*np.cos(angle)))
            if mode=="measured":
                a.plot(range(1,10),travel,"o--",color=color,label=name)
                b.plot(clock,out[:,0],color=color,lw=.9,label=name)
            for i,(start,stop) in enumerate(zip(starts,stops)):
                steady = (clock[10:-10]>=start+1)&(clock[10:-10]<stop-.1)&spin
                cm = (contact[:,0]>=start+1)&(contact[:,0]<stop-.1)
                sample = (t>=start+1)&(t<stop-.1)
                err = np.interp(t[sample],clock,out[:,0])-gyro[sample]
                load = contact[cm,1:5]
                lateral = np.divide(abs(contact[cm,5:9]).sum(),load.sum())
                radius = np.linalg.norm(body[steady],axis=1)
                table.append([mode,name,i+1,travel[i],np.linalg.norm(xy[rest[i+1]]-xy[rest[i]]-(center[ends[i+1]]-center[ends[i]])),
                              np.sqrt(np.mean(err**2)),np.median(radius),np.percentile(radius,95),lateral,
                              contact[cm,9].sum()/(load.sum()+contact[cm,9].sum())])
                if mode=="balanced":
                    ax=axes.flat[i]
                    # Show body-frame pivot through the spin at 10 ms intervals.
                    moving = spin&(clock[10:-10]>=start)&(clock[10:-10]<stop)
                    for k in np.flatnonzero(moving)[::10]:
                        traces.append([name,i+1,round(clock[k+10]-start,4),*np.round(body[k],4),
                                       *np.round(xy[k+10]+offset[k],4)])
                    ax.plot(clock[10:-10][moving][::10]-start,body[moving,1][::10],color=color,lw=.85,label=name)
                    ax.set_title(f"{i+1} · {'CCW' if i%2==0 else 'CW'}")
    a.set(ylabel="Rest-to-rest translation (mm)",xlabel="Turn number",title="Measured wheel-speed replay · 42 mm")
    a.legend(ncol=3)
    gyro_time = t.copy()
    lo,hi = np.argmax(np.diff(t)),np.searchsorted(t,91.49)
    gyro_time[lo:hi+1] = np.linspace(t[lo],t[hi],hi-lo+1)
    b.plot(gyro_time,gyro,color="#232f40",lw=.85,ls=":",label="Hardware")
    b.set(xlim=(65,92),xlabel="Run time (s)",ylabel="Head gyro Z (°/s)",title="Fast turns 5–9")
    b.legend(ncol=3)
    native_test(native)
    for ax in (*axes.flat,a,b,native):ax.grid(alpha=.2)
    axes.flat[0].legend(fontsize=8)
    pivot.supxlabel("Seconds after spin command")
    pivot.supylabel("Instantaneous pivot left of drive center (mm)")
    pivot.suptitle("Balanced-wheel pivot throughout nine alternating turns")
    pivot.savefig(FIG/"balanced-pivot.png",dpi=160)
    fig.savefig(FIG/"pivot-transfer.png",dpi=160)
    with (DATA/"pivot-paths.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("contact turn seconds_from_command forward_from_center_mm left_from_center_mm pivot_world_x_mm pivot_world_y_mm".split())
        w.writerows(traces)
    with (DATA/"pivot-turns.csv").open("w") as f:
        w=csv.writer(f,lineterminator="\n")
        w.writerow("input contact turn travel_mm vector_error_mm steady_gyro_rmse_dps pivot_median_radius_mm pivot_p95_radius_mm load_weighted_abs_lateral_contact_mm other_normal_fraction".split())
        for mode,name,i,*values in table:w.writerow([mode,name,i,*np.round(values,5)])


if __name__=="__main__":
    main()
