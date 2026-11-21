"""Compare baseline-relative head IMU signals through measured wheel reversals."""
import csv
import gzip
import json

import matplotlib.pyplot as plt
import numpy as np

from replay_carry import fixture, here, run, values
from simulacrum.imu_filter import ImuFilter


def simulated(t, states, loaded):
    wheel = np.array([[s.get(side + "_wheel_speed_mmps", 0) for side in ("left", "right")] for s in states])
    initial = [{"head": states[0]["head_angle_rad"], "lift": states[0]["lift_height_mm"]}]
    samples = []
    filt = ImuFilter(values["accel_filter_cutoff_hz"], values["gyro_filter_cutoff_hz"],
                     values["imu_filter_second_pole"], values["sim_timestep_ms"]/1000)

    def observe(model, data, clock):
        sample = np.r_[data.sensor("accel").data, data.sensor("gyro").data]
        if not samples:
            filt.reset(sample)
        filt.update(sample)
        samples.append((clock, filt.output[0]*1000*values["accel_scale"][0], np.degrees(filt.output[4])))

    run(initial, t, wheel, 40.5, configure=lambda root: fixture(root, loaded), observe=observe)
    clock, accel, pitch = np.array(samples).T
    return np.column_stack([np.interp(t, clock, v) for v in (accel, pitch)])


def main():
    events = list(map(json.loads, gzip.open(here/"rotation-verified.jsonl.gz", "rt")))
    fig, axes = plt.subplots(3, 2, figsize=(10, 7.5), sharex=True, sharey="col")
    scores = []
    for row, (trial, load) in enumerate([(3,"loaded"), (3,"empty"), (4,"loaded")]):
        first = next(i for i,e in enumerate(events) if e["kind"]=="trial" and e["number"]==trial)
        seq = events[first:]
        def stamp(kind, action):
            return next(e["t"] for e in seq if e["kind"]==kind and e.get("action")==load+" "+action)
        start = stamp("begin","turn outward")-.35
        zero = stamp("pulse_begin","reverse")
        end = stamp("pulse_end","forward2")+.45
        rows = [e for e in seq if e.get("name")=="robot_state" and start<=e["t"]<=end]
        t = np.array([e["t"]-zero for e in rows])
        states = [e["data"] for e in rows]
        observed = np.array([(s["accel"].get("x",0),np.degrees(s["gyro"].get("y",0))) for s in states])
        pre = (t>=-.2)&(t<0)
        assert pre.sum() >= 2
        measured = observed-np.median(observed[pre],axis=0)
        window = t>=-.2
        signals = {"Measured":measured}
        for name, loaded in (("Empty model",False),("50 g model",True)):
            series = simulated(t, states, loaded)
            signals[name] = series-np.median(series[pre],axis=0)
        for j, unit in enumerate(("Acceleration X (mm/s²)", "Pitch rate Y (°/s)")):
            ax = axes[row,j]
            for label, data in signals.items():
                ax.plot(t[window],data[window,j],".-" if label=="Measured" else "--",label=label,ms=3)
            for action in ("forward","reverse2","forward2"):
                ax.axvline(stamp("pulse_begin",action)-zero,color=".85",lw=.6)
            ax.set_title(f"{['Improper engagement','Empty raised fork','Carry, seating unverified'][row]} · {unit}")
            for label, data in list(signals.items())[1:]+[("Constant baseline",np.zeros_like(measured))]:
                scores.append((trial,load,unit,label,np.sqrt(np.mean((data[t>=0,j]-measured[t>=0,j])**2))))
    axes[0,0].legend(fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("Seconds from first reverse command")
    fig.tight_layout()
    fig.savefig(here/"reversal-imu.png",dpi=150)
    with (here/"reversal-imu.csv").open("w") as out:
        writer=csv.writer(out,lineterminator="\n")
        writer.writerow(["trial","physical_condition","channel","prediction","rmse"])
        writer.writerows(scores)


if __name__=="__main__":
    main()
