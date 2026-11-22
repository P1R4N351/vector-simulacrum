import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
import zlib

from export_map import export


class OriginRegistration(unittest.TestCase):
    def test_reused_origin_requires_own_dock_anchor(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"atlas.sqlite"
            with sqlite3.connect(path) as db:
                db.execute("CREATE TABLE revisions(session,origin,utc,leaves)")
                db.execute("CREATE TABLE seated(session,origin,utc,x,y,yaw)")
                for session, centre, kind, stamp in [("first",5,2,1),("next",105,7,2)]:
                    leaves = zlib.compress(json.dumps([[centre,5,10,kind]]).encode())
                    db.execute("INSERT INTO revisions VALUES(?,1,?,?)",(session,stamp,leaves))
                db.execute("INSERT INTO seated VALUES('first',1,1,0,0,0)")
                db.commit()
                first = export(path)
                self.assertEqual(first["cells"], [[0,0,2,0]])
                self.assertEqual(first["unregistered_revisions"], 1)
                db.execute("INSERT INTO seated VALUES('next',1,3,100,0,0)")
                db.commit()
                aligned = export(path)
                self.assertEqual(aligned["cells"], [[0,0,7,1]])
                self.assertEqual(aligned["unregistered_revisions"], 0)
