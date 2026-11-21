"""Compare two collision decompositions against one fixed STL and acquisition."""
import csv
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

import numpy as np
from PIL import Image, ImageDraw
import fork_envelope
import free_pickup

stl, baseline, finer = (Path(a).resolve() for a in sys.argv[1:])
out = free_pickup.here
with TemporaryDirectory() as tmp, (out/"fork-resolution.csv").open("w") as stream:
    tmp=Path(tmp)
    for filename in ("pickup-path.csv", "pickup-sightings.csv", "pickup.tsv", "action-times.csv"):
        (tmp/filename).symlink_to(out/filename)
    free_pickup.here = fork_envelope.here = tmp
    writer=csv.writer(stream,lineterminator="\n")
    writer.writerow(["fixture","initialization","hull_only_mm2","over_1mm_area_mm2","slide_2p86_mm","max_center_mm","held_floor_fraction"])
    panels=[]
    for label,fixture in [("original",baseline),("finer",finer)]:
        fork_envelope.main(stl,fixture)
        with Image.open(tmp/"fork-envelope.png") as image:
            panels.append((label,image.convert("RGB")))
        with (tmp/"fork-envelope.csv").open() as f:
            areas={r["region"]:float(r["projected_area_mm2"]) for r in csv.DictReader(f)}
        sys.argv[1]=str(fixture)
        free_pickup.main()
        data=np.genfromtxt(tmp/"free-pickup.csv",delimiter=",",names=True,dtype=None,encoding=None)
        for name in dict.fromkeys(data["initialization"]):
            run=data[data["initialization"]==name]
            point=run[np.argmin(abs(run["time_s"]-2.86))]
            held=run[(run["time_s"]>6.5)&(run["time_s"]<8.3)]
            writer.writerow([label,name,areas["hull_only"],areas["over_1_mm"],
                round(np.hypot(point["cube_dx_mm"],point["cube_dy_mm"]),3),
                round(run["center_z_mm"].max(),3),held["touches_floor"].mean()])

width,height=panels[0][1].size
canvas=Image.new("RGB",(width,2*(height+50)),"white")
draw=ImageDraw.Draw(canvas)
for idx,(label,image) in enumerate(panels):
    y=idx*(height+50)
    draw.text((30,y+8),f"{label.capitalize()} fork decomposition",fill="black",font_size=27)
    canvas.paste(image,(0,y+50))
canvas.save(out/"fork-resolution.png")
