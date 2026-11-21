"""Build the replay fixture from the existing collision model and a local copy of forks7.stl."""

import sys
from pathlib import Path
import xml.etree.ElementTree as ET

repo = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(repo), str(repo / "experiments/righting")]
from printed_fork import add_printed_fork
from simulacrum import params

if __name__ == "__main__":
    tree = ET.parse(repo / "evidence/unit3-cube-contact-replay/fixture.xml")
    root = tree.getroot()
    pieces = add_printed_fork(
        root,
        root.find(".//body[@name='fork']"),
        Path(sys.argv[1]),
        "",
        params.values(params.load(model="vector_2")),
    )
    tree.write(sys.argv[2])
    print(f"{pieces} convex parts")
