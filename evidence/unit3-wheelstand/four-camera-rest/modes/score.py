"""Score camera delivery within the interval common to all four containers."""
import csv
import json
from pathlib import Path
import subprocess
import sys


def main(root):
    out = csv.writer(sys.stdout, lineterminator="\n")
    out.writerow(["trial","size","camera","common_s","file_packets","file_empty","packets","nonempty","full_raw","delivery_hz","max_valid_gap_s"])
    for directory in sorted(root.glob("*-?*x*")):
        trial, size = directory.name.split("-")
        width, height = map(int, size.split("x"))
        views = {}
        for path in sorted(directory.glob("*.nut")):
            info = subprocess.check_output(["ffprobe","-v","error","-show_entries","packet=pts_time,size",
                                            "-of","json",str(path)])
            views[path.stem] = [(float(p["pts_time"]), int(p["size"])) for p in json.loads(info)["packets"]]
        assert set(views) == {"A","B","Arducam","C270"}
        start = max(rows[0][0] for rows in views.values())
        end = min(rows[-1][0] for rows in views.values())
        for name, rows in views.items():
            overlap = [(t,n) for t,n in rows if start <= t <= end]
            times = [t for t,n in overlap if (n == width*height*2 if name=="C270" else n>0)]
            gaps = [b-a for a,b in zip(times,times[1:])]
            out.writerow([trial,size,name,round(end-start,3),len(rows),sum(n==0 for t,n in rows),len(overlap),sum(n>0 for t,n in overlap),
                          len(times) if name=="C270" else "",round(len(times)/(end-start),2),
                          round(max(gaps),4) if gaps else ""])


if __name__=="__main__":
    main(Path(sys.argv[1]))
