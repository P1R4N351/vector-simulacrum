"""Apply the previously defined one-sided fork cuts to the independent pickup recording."""
import csv
import importlib
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from xml.etree import ElementTree as ET

import matplotlib.pyplot as plt
import mujoco as mj

run = importlib.import_module("fork-rake")
from unilateral_fork import trim

source = Path(sys.argv[1]).resolve()
columns = ("dt_ms rake_deg slide_cross_s slide_2p59_mm slide_2p86_mm max_rise_mm "
           "held_rise_mm tilt_3p11_deg tilt_3p38_deg tilt_3p64_deg").split()
rows = []
with TemporaryDirectory() as folder:
    for side, cut in [(0,0),(1,-8),(-1,-8),(1,-16),(-1,-16)]:
        clip = source
        if side:
            clip = Path(folder)/"cut.xml"
            trim(source,clip,side,cut)
        tree = ET.parse(clip)
        run.require_printed_fork(tree)
        run.add_links(tree,run.repo)
        model = mj.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding="unicode"))
        for label,sighting in [("last",run.sights[-1]),("pre-command",run.sights[run.sights["time_s"]<0][-1])]:
            for result in run.replay(model,[0],report_damage=True,sighting=sighting):
                rows.append([side,cut,*result,label])
with (run.here/"pickup-tooth.csv").open("w") as f:
    w=csv.writer(f,lineterminator="\n")
    w.writerow(["side","cut_z_mm",*columns,"sighting"])
    w.writerows(rows)
fig,axes=plt.subplots(1,2,figsize=(9,4),layout='constrained')
for step in [1,.5]:
    for label in ["last","pre-command"]:
        sub=[dict(zip(columns,r[2:])) for r in rows if r[2]==step and r[-1]==label]
        for ax,key in zip(axes,["slide_2p86_mm","max_rise_mm"]):
            ax.plot(range(5),[s[key] for s in sub],"o-" if step==1 else "x--",label=f"{label}, {step:g} ms")
for ax in axes:
    ax.set_xticks(range(5),["Intact","Left −8","Right −8","Left −16","Right −16"],rotation=25)
    ax.set_xlabel("Cut height relative to wrist (mm)")
axes[0].axhline(27.199,ls=":",color="k",label="Overhead observation")
axes[0].set_ylabel("Planar travel at +2.86 s (mm)")
axes[1].set_ylabel("Maximum cube center rise (mm)")
axes[0].legend()
fig.savefig(run.here/"pickup-tooth.png",dpi=150)
