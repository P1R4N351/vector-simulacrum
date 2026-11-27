"""Test temporal and cross-axis structure within three-sample cube packets."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from axis_noise import quiet_packets
from stationary_noise import observations, simulate, v

HERE = Path(__file__).resolve().parent


def stats(packets):
    one = np.diff(packets,axis=1)
    two = packets[:,2]-packets[:,0]
    ratio = np.sqrt(np.mean(two**2,axis=0)/np.mean(one**2,axis=(0,1)))
    corr = np.corrcoef(one.reshape(-1,3).T)
    return ratio,corr


def main():
    rest = observations("resting")
    sideways = np.genfromtxt(HERE.parent/"unit3-wheelstand/R8/settled.csv.gz",delimiter=",",names=True).reshape(-1,3)
    assert np.all(sideways["slot"]==[0,1,2]) and np.all(sideways["sec"]==sideways["sec"][:,0,None])
    side = np.stack([sideways[k] for k in "xyz"],axis=-1)/8192
    groups = {"Upright fit":rest,"Undock check":observations("undock-check"),
              "Before D1 lifts":quiet_packets("D1"),"Before D2 lifts":quiet_packets("D2"),"Side rest":side}
    # A correlation model for increments, estimated from one physical session.
    # Projection is diagnostic only; it does not change the packet encoder.
    _,fitted = stats(rest)
    rows = []
    axes3 = np.triu_indices(3,1)
    fig,axes = plt.subplots(1,2,figsize=(10,4),layout="constrained")
    for label,packets in groups.items():
        force = packets.mean(axis=(0,1))/v["cube_accel_scale"]
        predicted = np.concatenate([simulate(v["cube_accel_noise_density_ug"],force,seed,
                                    v["cube_accel_word_format"],v["cube_accel_noise_axis_scale"]) for seed in range(4)])
        ratio,corr = stats(packets)
        base_ratio,base_corr = stats(predicted)
        rows.append([label,len(packets),*ratio,*base_ratio,*corr[axes3],*base_corr[axes3],
                     np.sqrt(np.mean((corr[axes3]-base_corr[axes3])**2)),
                     np.sqrt(np.mean((corr[axes3]-fitted[axes3])**2))])
        axes[0].plot(range(3),ratio,"o-",label=label)
        axes[1].plot(range(3),corr[axes3],"o-",label=label)
    axes[0].plot(range(3),base_ratio,"k--",label="Canonical (side rest)")
    axes[1].plot(range(3),base_corr[axes3],"k--")
    axes[0].set(xticks=range(3),xticklabels=list("XYZ"),ylabel="Two-slot / one-slot RMS increment")
    axes[1].set(xticks=range(3),xticklabels=["XY","XZ","YZ"],ylabel="Correlation of simultaneous increments")
    axes[0].legend(fontsize=7)
    fig.savefig(HERE/"noise-increments.png",dpi=160)
    with (HERE/"noise-increments.csv").open("w") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow("session packets ratio_x ratio_y ratio_z sim_ratio_x sim_ratio_y sim_ratio_z corr_xy corr_xz corr_yz sim_corr_xy sim_corr_xz sim_corr_yz canonical_corr_rmse fitted_corr_rmse".split())
        writer.writerows(rows)
    packets=groups["Undock check"]
    peak=np.max(np.linalg.norm(np.diff(packets,axis=1),axis=2),axis=1)
    ranking=np.argsort(peak)[::-1]
    with (HERE/"noise-increment-sensitivity.csv").open("w") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow(["excluded_highest_packets","packets_remaining","corr_xy","corr_xz","corr_yz"])
        for count in (0,1,3,6,12):
            keep=np.ones(len(packets),dtype=bool)
            keep[ranking[:count]]=False
            writer.writerow([count,int(sum(keep)),*stats(packets[keep])[1][axes3]])
    print("Largest four packet increments at zero-based packet indices",ranking[:4])
    for r in rows:
        print(r[0],r[1],np.round(r[2:5],2),np.round(r[8:11],2),np.round(r[-2:],3))


if __name__=="__main__":
    main()
