"""Passively persist native unit 3 map revisions. Runs on the daughter pod."""
import hashlib
import json
import sqlite3
import sys
import time
import uuid
import zlib
from pathlib import Path

from anki_vector import Robot
from vector_slam_maze.navmap_atlas import extract_quadtree_leaves


def observe(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    db = sqlite3.connect(path)
    db.execute("PRAGMA journal_mode=WAL")
    db.execute("""CREATE TABLE IF NOT EXISTS revisions
        (session TEXT, origin INTEGER, utc REAL, leaves BLOB)""")
    db.execute("""CREATE TABLE IF NOT EXISTS seated
        (session TEXT, origin INTEGER, utc REAL, x REAL, y REAL, yaw REAL,
         PRIMARY KEY(session,origin))""")
    session = uuid.uuid4().hex
    with Robot("0a1b2c3d", ip="192.0.2.10", behavior_control_level=None,
               enable_nav_map_feed=True) as robot:
        last_map = last_digest = None
        last_frame = time.monotonic()
        while True:
            grid = robot.nav_map.latest_nav_map
            if grid is not None and grid is not last_map:
                origin = int(grid.origin_id)
                stamp = time.time()
                pose = robot.pose
                leaves = extract_quadtree_leaves(grid)
                payload = json.dumps([[p.cx_mm, p.cy_mm, p.size_mm, p.content]
                                      for p in leaves if p.content], separators=(",", ":")).encode()
                digest = origin, hashlib.sha256(payload).digest()
                with db:
                    if digest != last_digest:
                        db.execute("INSERT INTO revisions VALUES(?,?,?,?)",
                                   (session, origin, stamp, zlib.compress(payload)))
                        print(f"origin {origin}, {len(leaves)} native leaves", flush=True)
                    if (pose and pose.origin_id == origin and robot.status.is_on_charger
                            and not robot.status.are_wheels_moving and not robot.status.is_picked_up):
                        db.execute("INSERT OR IGNORE INTO seated VALUES(?,?,?,?,?,?)",
                                   (session, origin, stamp, pose.position.x, pose.position.y,
                                    pose.rotation.angle_z.radians))
                last_digest, last_map = digest, grid
                last_frame = time.monotonic()
            if time.monotonic() - last_frame > 45:
                raise TimeoutError("No fresh firmware map for 45 seconds")
            time.sleep(0.5)


if __name__ == "__main__":
    observe(Path(sys.argv[1]))
