"""Fit one brake-specific timing term to the original stop; test on the separate repeat session."""
import csv
import json
from pathlib import Path
import numpy as np
from replay_measured_turn import run

ROOT = Path(__file__).resolve().parent / "evidence"
DATA = ROOT / "hw-rec-20261119"
BASE_LAG = .017


def segments(path):
    events = [json.loads(line) for line in path.read_text().splitlines()]
    for i, cmd in enumerate(events):
        if cmd.get("name") != "wheels":
            continue
        end = next(r for r in events[i+1:] if r.get("name") == "stop")
        start, stop = cmd["t"], end["issued_t"]
        rows = [r for r in events if r["k"] == "s" and start-.4 <= r["t"] <= stop+.65]
        t = np.array([r["t"]-stop for r in rows])
        obs = np.degrees([r["gyro"][2] for r in rows])
        wheels = np.array([[r["wl"],r["wr"]] for r in rows])
        all_t = np.array([r["t"]-stop for r in events if r["k"] == "s"])
        gap = max(np.diff(all_t)[(all_t[:-1] < .4) & (all_t[1:] > -.1)])
        yield cmd["label"], rows, t, obs, wheels, start-stop, gap


def candidate(t, clock, gyro, extra):
    # Transition only after stop issue. Continuous time warp with nonnegative slope.
    return np.interp(t-BASE_LAG-extra*np.clip(t/.12,0,1),clock,gyro)


def rms(x):
    return np.sqrt(np.mean(x*x))


def main():
    import matplotlib.pyplot as plt
    original = next(segments(DATA / "unit2-ccw100.jsonl"))
    cases = [original, *segments(DATA / "unit2-repeat-stops.jsonl")]
    fig, axes = plt.subplots(2,len(cases),figsize=(16,5.8),sharex=True,sharey="row")
    all_rows = []
    for i,(label,rows,t,y,w,start,gap) in enumerate(cases):
        sign = 1 if "ccw" in label else -1
        before = after = np.full_like(t, np.nan)
        speed = abs(w[:,1]-w[:,0])/2
        window = (t >= 0) & (t <= .45)
        steady = (t >= t[np.flatnonzero((t>=start) & (np.min(abs(w),axis=1)>=90))[0]]) & (t<0)
        if gap <= .18:
            clock,out = run(rows,t,w,42,dense=True)
            gyro=out[:,0]
            before=candidate(t,clock,gyro,0)
            if i==0:
                grid=np.arange(101)*.001
                extra=min(grid,key=lambda lag:rms((candidate(t,clock,gyro,lag)-y)[window]))
                print("Original stop fit, additional delay (ms)", round(extra*1000))
            after=candidate(t,clock,gyro,extra)
            print(label, "brake n, RMS before/after, steady RMS before/after",sum(window),
                  *(round(x,2) for x in [rms((before-y)[window]),rms((after-y)[window]),
                                          rms((before-y)[steady]),rms((after-y)[steady])]))
        else:
            print(label, "unscored; gap across stop (s)",round(gap,3))
        view=(t>=-.25)&(t<=.5)
        for j,trace,color,style,legend in [(0,y*sign,'#1c2635','o-','Measured gyro'),
                                          (0,before*sign,'#b46243','--','17 ms baseline'),
                                          (0,after*sign,'#008074','-','Brake candidate'),
                                          (1,speed,'#425d99','o-','Wheel half-difference')]:
            axes[j,i].plot(t[view],trace[view],style,color=color,lw=1.6,ms=3,label=legend)
        if gap > .18:
            axes[0,i].text(.5,.45,"Stop not observed",ha="center",transform=axes[0,i].transAxes)
        axes[0,i].set_title(("Original CCW" if i==0 else label.upper().replace('_',' '))+
                            ("\nTelemetry gap" if gap>.18 else ""))
        for ax in axes[:,i]:
            ax.axvline(0,color=".55",lw=.8);ax.grid(alpha=.18);ax.spines[['top','right']].set_visible(False)
        for row in zip(t,w[:,0],w[:,1],y,before,after):
            all_rows.append([label,*row])
    axes[0,0].set_ylabel("Direction-aligned gyro Z (°/s)")
    axes[1,0].set_ylabel("Encoder speed (mm/s)")
    fig.supxlabel("Seconds from stop-command issue")
    fig.legend(*axes[0,0].get_legend_handles_labels(),loc="upper center",ncol=3,bbox_to_anchor=(.5,.935))
    fig.suptitle("Does the body keep turning as the wheels brake?",weight="bold",fontsize=16)
    fig.subplots_adjust(top=.77,bottom=.13,hspace=.16,wspace=.12)
    fig.savefig(ROOT/"physics-plots"/"repeated-braking.png",dpi=150)
    with (DATA/"braking-predictions.csv").open("w",newline="") as f:
        writer=csv.writer(f)
        writer.writerow(["turn","time_from_stop_s","left_mmps","right_mmps","gyro_dps","baseline_dps","candidate_dps"])
        writer.writerows(all_rows)


if __name__ == "__main__":
    main()
