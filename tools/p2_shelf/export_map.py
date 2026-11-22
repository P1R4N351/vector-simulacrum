"""Export a dock-registered history of native map leaves for the simulator.

Run on the pod with plain Python. Atomic SQLite read transaction. Each anchored
origin contributes its latest observed terrain at each 10 mm cell, including
previous coverage no longer present in the active map.
"""
import json
import math
import sqlite3
import sys
import zlib
from pathlib import Path


def export(db_path):
    with sqlite3.connect(db_path.as_uri() + "?mode=ro", uri=True) as db:
        db.execute("BEGIN")
        registered = db.execute("""SELECT r.utc,r.leaves,s.x,s.y,s.yaw FROM revisions r
            JOIN seated s USING(session,origin) ORDER BY r.utc,r.rowid""")
        terrain, conflicts = {}, set()
        latest, revisions = None, 0
        semantic = []
        for latest, compressed, ox, oy, yaw in registered:
            c, s = math.cos(yaw), math.sin(yaw)
            semantic = []
            for x, y, width, kind in json.loads(zlib.decompress(compressed)):
                # Rasterize in the docked robot's frame by testing cell centres
                # against the original rotated leaf. Never spread cliffs by bounding box.
                dx, dy = x-ox, y-oy
                cx, cy = c*dx+s*dy, -s*dx+c*dy
                if kind not in (2, 7):
                    semantic.append([round(cx, 2), round(cy, 2), width, kind, round(-yaw, 6)])
                    continue
                radius = width*(abs(c)+abs(s))/2
                for i in range(math.floor((cx-radius)/10), math.ceil((cx+radius)/10)):
                    for j in range(math.floor((cy-radius)/10), math.ceil((cy+radius)/10)):
                        a, b = (i+.5)*10-cx, (j+.5)*10-cy
                        if abs(c*a-s*b) > width/2 or abs(s*a+c*b) > width/2:
                            continue
                        if (i,j) in terrain and terrain[i,j] != kind:
                            conflicts.add((i,j))
                        terrain[i,j] = kind
            revisions += 1
        total = db.execute("SELECT count(*) FROM revisions").fetchone()[0]
    return dict(cell_mm=10, frame="seated_robot", latest_utc=latest, revisions=revisions,
                unregistered_revisions=total-revisions, latest_objects_and_visibility=semantic,
                cells=[[i,j,k,int((i,j) in conflicts)] for (i,j),k in sorted(terrain.items())])


if __name__ == "__main__":
    data = json.dumps(export(Path(sys.argv[1]).resolve()), separators=(",", ":")) + "\n"
    if len(sys.argv) == 3:
        dest = Path(sys.argv[2])
        temporary = dest.with_suffix(".tmp")
        temporary.write_text(data)
        temporary.replace(dest)
    else:
        print(data, end="")
