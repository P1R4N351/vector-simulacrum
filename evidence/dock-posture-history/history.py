"""Join overlapping listeners once, using the continuation from its first packet."""
import csv
import gzip
import json
import math
from datetime import datetime

def read(name):
    with gzip.open("../unit3-w34-return/"+name+".csv.gz","rt") as f:
        return list(csv.DictReader(f))

first, second = read("states"), read("continuation")
on = next(i for i in range(1,len(first)) if int(first[i]["status"]) & 4096 and not int(first[i-1]["status"]) & 4096)
off = next(r for r in second if not int(r["status"]) & 4096)
zero = datetime.fromisoformat(off["utc"])
rows = [r for r in first[on:] if r["utc"] < second[0]["utc"]] + second
out=[]
for r in rows:
    t=(datetime.fromisoformat(r["utc"])-zero).total_seconds()
    if t > 0.8:
        break
    out.append([t,float(r["left_mmps"]),float(r["right_mmps"]),math.degrees(float(r["head_rad"])),float(r["lift_mm"]),float(r["pitch_rad"])])
assert all(b[0]>a[0] for a,b in zip(out,out[1:]))
print(json.dumps(out))
