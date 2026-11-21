"""Write the generated MJCF to a file or to stdout..

    python3 build_mjcf.py                       nominal robot on the table, to stdout
    python3 build_mjcf.py --out vector.xml
    python3 build_mjcf.py --seed 7 --out v7.xml  a robot drawn inside the parameter ranges
    python3 build_mjcf.py --set total_mass_g=160 --set table_length_mm=900

Needs no MuJoCo.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, List

sys.path.insert(0, str(Path(__file__).resolve().parent))

from simulacrum import mjcf, params  # noqa: E402

EXIT_OK, EXIT_USAGE = 0, 2


def _overrides(pairs: List[str]) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for pair in pairs:
        name, sep, text = pair.partition("=")
        if not sep or not name:
            raise ValueError(f"--set wants name=number, got {pair!r}")
        out[name] = float(text)
    assert len(out) <= len(pairs), "override count grew"
    return out


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="Generate the Vector simulacrum MJCF.")
    parser.add_argument("--out", help="file to write; stdout when omitted")
    parser.add_argument("--seed", type=int, help="draw ranged robot parameters from this seed")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=NUMBER", help="override one parameter")
    args = parser.parse_args(argv)
    doc = params.load()
    values = params.sample(doc, args.seed) if args.seed is not None else params.values(doc)
    try:
        for name, number in _overrides(args.set).items():
            if name not in values:
                raise KeyError(f"unknown parameter {name!r}")
            values[name] = number
        text = mjcf.build_mjcf(values)
    except (ValueError, KeyError, AssertionError) as err:
        print(f"error: {err}", file=sys.stderr)
        return EXIT_USAGE
    if args.out:
        Path(args.out).write_text(text, encoding="utf-8")
        print(f"wrote {len(text)} characters to {args.out}", file=sys.stderr)
    else:
        sys.stdout.write(text)
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
