"""Extract origin-matched image stamps and latest reported cube pose from the SDK logs."""
import csv
import gzip
import json
from pathlib import Path
import sys

here = Path(__file__).resolve().parent
decoder = json.JSONDecoder()
for trial in ("W4", "W8", "W12"):
    events = []
    for line in (Path(sys.argv[1])/f"p2wheel-{trial}.jsonl").read_text().splitlines():
        while line.strip():
            try:
                event,n = decoder.raw_decode(line.lstrip())
            except ValueError:
                break
            events.append(event)
            line = line.lstrip()[n:]
    start = next(e["t"] for e in events if e["kind"]=="begin" and e.get("action")=="pop_a_wheelie")
    sightings = [e["data"] for e in events if e["kind"]=="robot_observed_object" and e["data"].get("object_family")=="LIGHT_CUBE"]
    taps = [e["t"]-start for e in events if e["kind"]=="object_tapped" and e["t"]>start]
    rows = {}
    for event in events:
        t = event["t"]-start
        if event["kind"]!="robot_state" or not -.5 <= t <= min(taps+[5.5]):
            continue
        state = event["data"]
        stamp = state.get("last_image_time_stamp")
        pose = state["pose"]
        seen = [o for o in sightings if stamp and o["timestamp"]<=stamp and o["pose"].get("origin_id",0)==pose.get("origin_id",0)]
        if not seen or stamp in rows: continue
        o=max(seen,key=lambda o:o["timestamp"])
        rows[stamp]=[t,stamp,int(stamp==o["timestamp"]),stamp-o["timestamp"],state["head_angle_rad"],state["lift_height_mm"],
                     *[p.get(k,0) for p in (pose,o["pose"]) for k in ("x","y","z","q0","q1","q2","q3")]]
    with gzip.open(here/f"{trial}.csv.gz","wt") as f:
        writer=csv.writer(f);writer.writerow(["sec","stamp","seen","age_ms","head","lift"]+[prefix+k for prefix in ("r","c") for k in ("x","y","z","q0","q1","q2","q3")]);writer.writerows(rows.values())
    print(trial,len(rows),min(taps+[5.5]))
