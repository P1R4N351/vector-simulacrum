"""Write diagnostic variants of the exact canonical shelf file to a temporary directory."""
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulacrum.mjcf import _tip_equality_xml, _tip_supports_xml

dest = Path(sys.argv[1])
dest.mkdir(exist_ok=True, parents=True)
assets = ROOT / "web/public/robot/vector2"
v = json.loads((assets / "model.json").read_text())["values"]
for mode in ("stock", "box", "plane", "rear-free", "belly-free", "clearance", "driven"):
    tree = ET.parse(assets / "vector-free-shelf.xml")
    world = tree.find("worldbody")
    chassis = tree.find(".//geom[@name='chassis_geom']/..")
    belly = chassis.find("geom[@name='chassis_geom']")
    if mode in ("box", "plane"):
        for g in world.findall("geom"):
            if g.get("name", "").startswith("shelf_"):
                world.remove(g)
        ET.SubElement(world, "geom", name="control_floor", type=mode, size=".3 .3 .01",
                      pos="0 0 -.01" if mode == "box" else "0 0 0", contype="0", conaffinity="1")
    if mode == "rear-free":
        for side in "lr":
            chassis.find(f"geom[@name='tip_rear_{side}']").set("contype", "0")
    if mode == "belly-free":
        belly.set("contype", "0")
    if mode == "clearance":
        # Raise only the lower face 0.25 mm. Chassis mass/inertia are explicit in this scene.
        for field, change in [("pos", .000125), ("size", -.000125)]:
            xyz = list(map(float, belly.get(field).split()))
            xyz[2] += change
            belly.set(field, " ".join(map(str, xyz)))
    if mode == "driven":
        for g in chassis.findall("geom"):
            if g.get("name", "").startswith("tip_"):
                chassis.remove(g)
        driven = dict(v, tip_supports_driven=1)
        chassis.extend(ET.fromstring("<parts>" + _tip_supports_xml(driven) + "</parts>"))
        stiff = 'solref="%g 1"' % (.002 * v["sim_timestep_ms"])
        tree.find("equality").extend(ET.fromstring("<parts>" + _tip_equality_xml(driven, stiff) + "</parts>"))
    tree.write(dest / (mode + ".xml"), encoding="unicode")
