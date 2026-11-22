"""Build local Vector 2 web scenes using Dauler's registered printed fork."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from fetch_print_parts import DEST, PART_SHA256

WEB = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WEB.parent / "experiments/righting"))
from printed_fork import add_printed_fork


def export(stl):
    if hashlib.sha256(stl.read_bytes()).hexdigest() != PART_SHA256["forks7.stl"]:
        raise ValueError("Expected the registered forks7.stl used in the cube replay")
    directory = WEB / "public/robot/vector2"
    model = json.loads((directory / "model.json").read_text())
    output = directory / "printed"
    output.mkdir(exist_ok=True)
    template = None
    for scene, key in (("vector", "xml_sha256"), ("vector-contact42", "trial_xml_sha256"),
                       ("vector-free", "vector-free_sha256"), ("vector-contact42-free", "vector-contact42-free_sha256")):
        source = (directory / f"{scene}.xml").read_bytes()
        if hashlib.sha256(source).hexdigest() != model["source"][key]:
            raise ValueError(f"Re-export the base scene {scene} before generating the printed fork")
        root = ET.fromstring(source)
        fork = root.find(".//body[@name='lift_fork']")
        proxies = [geom for geom in fork.findall("geom")]
        signature = [dict(geom.attrib) for geom in proxies]
        if template is None:
            count = add_printed_fork(root, fork, stl, "", model["values"])
            template = signature, list(root.findall("./asset/mesh"))[-count:], fork.findall("geom")[-count:]
        else:
            expected, meshes, geoms = template
            if signature != expected:
                raise ValueError(f"Fork differs from the registration template in {scene}")
            asset = root.find("asset")
            if asset is None:
                asset = ET.SubElement(root, "asset")
            asset.extend(copy.deepcopy(meshes))
            fork.extend(copy.deepcopy(geoms))
            for proxy in proxies:
                proxy.attrib.update(contype="0", conaffinity="0", rgba="0 0 0 0")
        data = ET.tostring(root, encoding="utf-8")
        (output / f"{scene}.xml").write_bytes(data)
        model["source"][key] = hashlib.sha256(data).hexdigest()
    (output / "model.json").write_text(json.dumps(model, indent=2) + "\n")
    print(f"Exported four scenes with {len(template[1])} contact hulls to {output}")


if __name__ == "__main__":
    export(Path(sys.argv[1]) if len(sys.argv) > 1 else DEST / "forks7.stl")
