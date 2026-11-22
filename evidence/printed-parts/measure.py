"""Measure every registered printed part and print what the model takes from it.

    python3 evidence/printed-parts/measure.py [--fetch] [--json]

Needs the parts in the cache (simulacrum/printed_parts.py; --fetch downloads them from their sources,
checked against the registered hashes). No MuJoCo. The README beside this file quotes this output.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from simulacrum import params, printed_parts as pp  # noqa: E402


def row(v: Dict[str, Any], name: str) -> Dict[str, Any]:
    kind = pp.PARTS[name]["kind"]
    report = pp.part_report(v, (name,))
    part = report[kind]
    out: Dict[str, Any] = {"part": name, "kind": kind, "material": part["material"], "fill": part["fill"]}
    if kind == "fork":
        out.update(mass_g=part["mass_g"], volume_mm3=part["volume_mm3"], com_mm=[float(c) for c in part["com_mm"]],
                   boxes=len(part["boxes"]), voxel_volume_mm3=part["voxel_volume_mm3"],
                   overlap=part["registration_overlap"], runner_up=part["registration_runner_up"],
                   stock_fork_mass_g=part["replaces_mass_g"])
    elif kind == "rims":
        for end in ("front", "rear"):
            out[end] = {"diameter_mm": 2 * part[end]["outer_mm"], "width_mm": part[end]["width_mm"],
                        "mass_g": part[end]["mass_g"], "axial_inertia_gmm2": float(part[end]["inertia_gmm2"][1, 1])}
    else:
        band = part["left"]
        out.update(inner_radius_mm=band["inner_mm"], outer_radius_mm=band["outer_mm"], thickness_mm=band["thickness_mm"],
                   width_mm=band["width_mm"], mass_g=band["mass_g"], share_g=band["share_g"],
                   same_both_sides=part["left"]["volume_mm3"] == part["right"]["volume_mm3"])
    out["rolling_radius_mm"] = report["rolling_radius_mm"]["left"]
    return out


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--fetch", action="store_true", help="download missing parts from their sources")
    parser.add_argument("--json", action="store_true", help="one JSON object per part")
    args = parser.parse_args(argv)
    v = params.values(params.load())
    rows = []
    for name in pp.PARTS:
        if args.fetch:
            try:
                pp.load(name)
            except FileNotFoundError:
                pp.fetch_part(name)
        rows.append(row(v, name))
    stock = pp.part_report(v, ())["stock_tread"]
    if args.json:
        for r in rows + [{"stock_tread": stock}]:
            print(json.dumps(r, sort_keys=True))
        return 0
    for r in rows:
        print(json.dumps(r, sort_keys=True, default=float))
    print("stock tread (assumed):", json.dumps(stock, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
