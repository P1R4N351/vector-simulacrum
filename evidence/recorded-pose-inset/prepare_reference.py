"""Recreate the physics used for this offline experiment in a fresh directory."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def write_table(destination, filename, key, updates):
    table = json.loads((ROOT / filename).read_text())
    table["sources"]["POSEFIT"] = {"what": "Numerical reference for bbec512a1212"}
    for name, value, low, high, unit in updates:
        table[key][name] = {"value":value, "low":low, "high":high, "unit":unit,
                            "provenance":"assumed" if key == "parameters" else "fitted", "kind":"robot", "source":"POSEFIT: REC head sweep on P2Y8, reference bbec512a1212"}
    (destination / filename).write_text(json.dumps(table))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    dest = parser.parse_args().destination.resolve()
    dest.mkdir(parents=True, exist_ok=False)
    shutil.copytree(ROOT / "simulacrum", dest / "simulacrum", ignore=shutil.ignore_patterns("__pycache__"))
    subprocess.run(["git", "apply", str(HERE / "reference-model.patch")], cwd=dest, check=True)
    write_table(dest, "parameters.json", "parameters", [
        ("accel_scale", [1.,1.,1.], [.96]*3, [1.04]*3, "1")])
    write_table(dest, "parameters-vector2.json", "overrides", [
        ("accel_scale", [1.014,1.,1.], [1.008,.96,.96], [1.021,1.04,1.04], "1"),
        ("accel_bias_mg", [13.2,4.,-18.1], [3.2,-6.,-28.1], [23.2,14.,-8.1], "mg"),
        ("imu_pitch_offset_deg", -.05, -.55, .45, "deg")])


if __name__ == "__main__":
    main()
