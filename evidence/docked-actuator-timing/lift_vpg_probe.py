"""Compare compiled upstream ramp targets with the canonical lift ramp."""
import argparse
import csv
import gzip
import hashlib
import json
import subprocess
import tempfile
from math import degrees
from pathlib import Path
from statistics import median

import matplotlib.pyplot as plt
from lift_crossings import ROOT, crossing
from simulacrum import params
from simulacrum.robot import VectorSim


def build(source, work):
    for ext, digest in [("cpp","a46d6ba4781870fc1979b051b4bb049bcbb4d15b7ea757eedb0b30064774f127"),
                        ("h","eec1dae83f805186d0bafa3c5f08e77ed121437180066da1bbc9ede8f7b0eef4")]:
        file = "velocityProfileGenerator."+ext
        data = (source/file).read_bytes()
        assert hashlib.sha256(data).hexdigest()==digest, file
        (work/file).write_bytes(data)
    # Only platform includes and logging are stubbed. Preserve upstream failure returns.
    for include in ("anki/common/constantsAndMacros.h","anki/cozmo/robot/hal.h",
                    "anki/cozmo/robot/logging.h","messages.h"):
        file = work/include
        file.parent.mkdir(parents=True,exist_ok=True)
        file.touch()
    (work/"anki/common/constantsAndMacros.h").write_text(
        "#include <cassert>\n#define ABS(x) fabsf(x)\n#define AnkiAssert(x,...) assert(x)\n"
        "#define AnkiConditionalWarnAndReturnValue(x,v,...) do { if (!(x)) return v; } while(0)\n"
        "#define AnkiInfo(...) ((void)0)\n#define AnkiWarn(...) ((void)0)\n")
    exe = work/"profile"
    subprocess.run(["c++","-std=c++11","-I",str(work),str(work/"velocityProfileGenerator.cpp"),
                    str(ROOT/"lift_vpg_probe.cpp"),"-o",str(exe)],check=True)
    return exe


class Probe(VectorSim):
    sequence = None

    def _ramp_commands(self):
        super()._ramp_commands()
        if self.sequence is not None:
            self._lift_cmd = next(self.sequence,self._lift_target)


def run(exe):
    with gzip.open(ROOT.parent/"unit3-lift-history/physical.jsonl.gz","rt") as stream:
        events = [json.loads(line) for line in stream]
    states = [r for r in events if r["kind"]=="state"]
    cues = [r for r in events if r["kind"]=="command"]
    with (ROOT/"lift-crossings.csv").open() as stream:
        reference = {(int(r["command"]),float(r["fraction"])):r for r in csv.DictReader(stream)
                     if r["recording"]=="Unit 3 timed"}
    v = params.values(params.load(model="vector_2"))
    rows = []
    endpoints = []
    for num,cue in enumerate(cues,1):
        if (num,.1) not in reference:
            continue
        pose = next(r for r in reversed(states) if r["t"]<cue["t"])
        h = pose["lift"]
        target = 32+60*cue["pos"]
        sim = Probe(v,head_deg=degrees(pose["head"]),lift_mm=h,charger_contact=True)
        assert sim.dt==.005
        start = sim._lift_cmd
        assert sim.command_lift_height(target,duration_s=.7)
        targets = subprocess.check_output([exe,str(start),str(sim._lift_target),".7"],text=True)
        sim.sequence = iter(map(float,targets.splitlines()))
        trace = [(0,sim.lift_height_mm())]
        baseline = VectorSim(v,head_deg=degrees(pose["head"]),lift_mm=h,charger_contact=True)
        baseline.command_lift_height(target,duration_s=.7)
        flags = [None,None]
        max_mm = 0.0
        for _ in range(360):
            sim.step()
            baseline.step()
            max_mm = max(max_mm,abs(baseline.lift_height_mm()-sim.lift_height_mm()))
            for i,model in enumerate((baseline,sim)):
                if flags[i] is None and model._lift_in_pos:
                    flags[i] = model.tick*model.dt
            trace.append((sim.tick*sim.dt,sim.lift_height_mm()))
        assert all(t is not None for t in flags)
        endpoints.append([num,cue["block"],*flags,max_mm])
        for frac in (.1,.5,.9):
            ref = reference[num,frac]
            stock = crossing(trace,h+(target-h)*frac,1 if target>h else -1)
            assert stock
            rows.append([num,cue["block"],frac,float(ref["interpolated_s"]),float(ref["sim_s"]),stock[1]])
    with (ROOT/"lift-vpg.csv").open("w") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow("command block fraction measured_s canonical_s upstream_s".split())
        writer.writerows(rows)
    with (ROOT/"lift-vpg-endpoints.csv").open("w") as stream:
        writer=csv.writer(stream,lineterminator="\n")
        writer.writerow("command block canonical_settled_s upstream_settled_s max_height_difference_mm".split())
        writer.writerows(endpoints)
    print("Max height change mm",max(r[4] for r in endpoints),"max settled time change ms",
          max(1000*abs(r[3]-r[2]) for r in endpoints))
    fig,ax=plt.subplots(figsize=(7,4),layout="constrained")
    for i,label in [(4,"Canonical"),(5,"Upstream target generator")]:
        ax.scatter(range(len(rows)),[1000*(r[3]-r[i]) for r in rows],
                   facecolors="none" if i==4 else "C1",edgecolors="C0" if i==4 else "C1",
                   s=55 if i==4 else 9,label=label)
    ax.axhline(0,color=".65",lw=.8)
    ax.set(xlabel="Encoder crossings in recording order (10%, 50%, 90% per stroke)",
           ylabel="Measured − simulated crossing (ms)")
    ax.legend()
    fig.savefig(ROOT/"lift-vpg.png",dpi=160)
    for blocks in ((0,1),(2,3)):
        group=[r for r in rows if r[1] in blocks]
        print(blocks,[round(1000*median(abs(r[3]-r[i]) for r in group),2) for i in (4,5)])
    print("Maximum change in ms",max(1000*abs(r[4]-r[5]) for r in rows))


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("source",type=Path,help="Directory with the two upstream velocityProfileGenerator sources")
    with tempfile.TemporaryDirectory() as tmp:
        run(build(parser.parse_args().source,Path(tmp)))
