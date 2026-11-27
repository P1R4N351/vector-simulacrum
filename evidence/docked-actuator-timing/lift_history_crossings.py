"""Compare two kinds of lift initialization on the same measured threshold crossings."""
import csv
import gzip
import json
from math import degrees
from statistics import median

import matplotlib.pyplot as plt
from lift_crossings import ROOT, crossing
from simulacrum import params
from simulacrum.robot import VectorSim


def main():
    with gzip.open(ROOT.parent/"unit3-lift-history/physical.jsonl.gz", "rt") as stream:
        events = [json.loads(line) for line in stream]
    states = [r for r in events if r["kind"] == "state"]
    cues = [r for r in events if r["kind"] == "command"]
    with (ROOT/"lift-crossings.csv").open() as stream:
        reference = {(int(r["command"]),float(r["fraction"])):r for r in csv.DictReader(stream)
                     if r["recording"]=="Unit 3 timed"}
    values = params.values(params.load(model="vector_2"))
    rows = []
    absolute = []
    for block in range(4):
        commands = [(i,c) for i,c in enumerate(cues,1) if c["block"]==block]
        origin = commands[0][1]["t"]
        start = next(r for r in reversed(states) if r["t"] < origin)
        sim = VectorSim(values,head_deg=degrees(start["head"]),lift_mm=start["lift"],charger_contact=True)
        for j,(num,c) in enumerate(commands):
            t = c["t"]
            tick = round((t-origin)/sim.dt)
            assert sim.tick == tick
            target = 32+60*c["pos"]
            h = next(r["lift"] for r in reversed(states) if r["t"]<t)
            initial = sim.lift_height_mm()
            assert sim.command_lift_height(target,duration_s=.7)
            end = commands[j+1][1]["t"] if j+1<len(commands) else t+1.9
            trace = [(0,initial)]
            for _ in range(round((end-origin)/sim.dt)-tick):
                sim.step()
                trace.append(((sim.tick-tick)*sim.dt,sim.lift_height_mm()))
            if c["phase"]=="measure":
                observed = [(r["t"]-t,r["lift"]) for r in states if t<=r["t"]<end]
                for height in (80,84,88):
                    measured = crossing(observed,height,1)
                    predicted = crossing(trace,height,1)
                    assert measured and predicted
                    absolute.append([block,c["prep"],height,*measured,predicted[1]])
            for frac in (.1,.5,.9):
                ref = reference.get((num,frac))
                if ref is None:
                    continue
                threshold = h + frac*(target-h)
                edge = crossing(trace,threshold,1 if target>h else -1)
                rows.append([num,block,c["prep"],c["phase"],frac,h,initial,float(ref["interpolated_s"]),
                             float(ref["sim_s"]),edge[1] if edge else None])
    with (ROOT/"lift-history-crossings.csv").open("w") as stream:
        writer = csv.writer(stream,lineterminator="\n")
        writer.writerow("command block prep phase fraction actual_start_mm sim_start_mm measured_s reset_s continuous_s".split())
        writer.writerows(rows)
    with (ROOT/"lift-history-heights.csv").open("w") as stream:
        writer = csv.writer(stream,lineterminator="\n")
        writer.writerow("block prep height_mm measured_lo_s measured_s measured_hi_s continuous_s".split())
        writer.writerows(absolute)
    for height in (80,84,88):
        for block in range(4):
            a,b=sorted([r for r in absolute if r[0]==block and r[2]==height],key=lambda r:r[1])
            print(height,block,[round(1000*v) for v in (b[3]-a[5],b[4]-a[4],b[5]-a[3],b[6]-a[6])])
    fig, axes = plt.subplots(1,2,figsize=(9,4.2),sharey=True,layout="constrained")
    for ax,prep,title in zip(axes,(.5,1),("Raise after raising","Raise after lowering")):
        for block in range(4):
            trace = [r for r in rows if r[1]==block and r[2]==prep and r[3]=="measure"]
            for idx,style,label in [(8,"--","Reset"),(9,"-","Continuous")]:
                ax.plot([100*r[4] for r in trace],[1000*(r[7]-r[idx]) if r[idx] is not None else float("nan") for r in trace],
                        style,color=f"C{block}",marker=".",label=f"Block {block+1} {label.lower()}" if ax==axes[1] else None)
        ax.axhline(0,color=".65",lw=.8)
        ax.set(title=title,xlabel="Final raise stroke (%)",xticks=[10,50,90])
    axes[0].set_ylabel("Measured − simulated crossing (ms)")
    axes[1].legend(fontsize=7,ncols=2)
    fig.savefig(ROOT/"lift-history-crossings.png",dpi=160)
    for idx in (8,9):
        for blocks in ((0,1),(2,3)):
            subset = [1000*abs(r[7]-r[idx]) for r in rows if r[1] in blocks and r[9] is not None]
            print(idx,blocks,len(subset),round(median(subset)))
    for row in rows:
        if row[9] is None: print("Missing crossing",row[:5])
    for block in range(4):
        pair = [[r for r in rows if r[3]=="measure" and r[1]==block and r[2]==prep] for prep in (.5,1)]
        print("Reversal minus continuation in milliseconds",block,
              [[round(1000*(b[idx]-a[idx])) if b[idx] is not None and a[idx] is not None else None for a,b in zip(*pair)] for idx in (7,8,9)])


if __name__=="__main__":
    main()
