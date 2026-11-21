"""Register Dauler's forks7.stl to the simulated wrist and preserve its open crossbar."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def add_printed_fork(
    root: ET.Element, fork: ET.Element, stl: Path, prefix: str, values: dict
) -> int:
    """Replace only the fork's contact surfaces, retaining its original mass and joint."""
    import coacd

    record = np.dtype(
        [("normal", "<f4", 3), ("triangle", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    data = stl.read_bytes()
    count = int.from_bytes(data[80:84], "little")
    if count == 0 or len(data) != 84 + count * record.itemsize:
        raise ValueError("Expected a binary forks7 STL")
    faces = np.frombuffer(data, offset=84, dtype=record)["triangle"]
    x, y, z = faces.astype(float).transpose(2, 0, 1)
    # Same print-bed to wrist registration as web/src/game/printparts.js.
    points = np.stack(
        (
            -z + values["lift_front_wrt_wrist_mm"] - 26.52,
            -x - 48.01,
            y
            + values["lift_xbar_top_wrt_wrist_mm"]
            - values["lift_xbar_height_mm"]
            + 36.79,
        ),
        axis=-1,
    )
    vertices, indices = np.unique(points.reshape(-1, 3), axis=0, return_inverse=True)
    pieces = coacd.run_coacd(
        coacd.Mesh(vertices, indices.reshape(-1, 3)), threshold=0.025, seed=0
    )
    if not pieces:
        raise ValueError("Fork decomposition returned no hulls")
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    reference = fork.find(f"geom[@name='{prefix}lift_fork_geom']")
    contact = {
        k: v
        for k, v in reference.attrib.items()
        if k not in ("name", "type", "pos", "size", "mass")
    }
    for geom in fork.findall("geom"):
        if geom.get("name", "").startswith(
            (prefix + "lift_fork_geom", prefix + "fork_side_")
        ):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
            geom.set("rgba", "0 0 0 0")
    for index, (points, faces) in enumerate(pieces):
        name = f"{prefix}lift_fork_geom_stl_{index}"
        ET.SubElement(
            asset,
            "mesh",
            name=name,
            vertex=" ".join(f"{v * 0.001:.9f}" for v in points.ravel()),
            face=" ".join(map(str, faces.ravel())),
        )
        ET.SubElement(
            fork, "geom", dict(contact, type="mesh", mesh=name, name=name, mass="0")
        )
    return len(pieces)
