"""Record one or more phases on your Vector and package the result as a submission.

A thin wrapper around hardware/record_barriered.py and contrib/kit/scrub.py. It
writes metadata.yaml from flags and prompts, runs the recorder for each named
phase, applies the privacy scrub, and reports whether the resulting folder
passes contrib/validator.py.

Exit codes: 0 ready to commit, 1 recording failed or scrub refused, 2 bad arguments.

Usage:
    python3 contrib/kit/record.py --unit-handle <handle>-<n> \\
        --vector-serial <8hex> --phases rest,headlift,spins,straights,actions
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
KIT = ROOT / "contrib" / "kit"
UNITS = ROOT / "contrib" / "units"
RECORDER = ROOT / "hardware" / "record_barriered.py"
VALID_PHASES = {"rest", "headlift", "spins", "straights", "actions", "reversal", "arcs"}
HANDLE_RX = re.compile(r"^[a-z0-9][a-z0-9-]{1,31}-[0-9]+$")
SERIAL_RX = re.compile(r"^[0-9a-f]{8}$")
MAX_PHASES = 8


def prompt(question: str, default: str = "") -> str:
    """Prompt the operator with a question; return the string they typed, stripped. Empty means default."""
    assert question, "a question is required"
    try:
        s = input(question + (f" [{default}]" if default else "") + ": ").strip()
    except EOFError:
        return default
    return s or default


def write_metadata(unit_dir: Path, hardware: str, firmware: str, mods: str,
                   licence: str, sign_off: str, contributor: str, surface: str) -> None:
    """Write metadata.yaml. Values are quoted; no newline injection."""
    assert unit_dir.is_dir(), f"{unit_dir} is not a directory"
    assert hardware in {"1.0", "2.0"}, "hardware_generation must be 1.0 or 2.0"
    def safe(v: str) -> str:
        return v.replace("\n", " ").replace('"', "")
    text = (
        f'hardware_generation: "{hardware}"\n'
        f'firmware_version: "{safe(firmware)}"\n'
        f'body_modifications: "{safe(mods)}"\n'
        f'recorded_at: "{safe_isodate()}"\n'
        f'licence: "{licence}"\n'
        f'sign_off: "{safe(sign_off)}"\n'
        f'contributor: "{safe(contributor)}"\n'
        f'surface: "{safe(surface)}"\n'
    )
    (unit_dir / "metadata.yaml").write_text(text, encoding="utf-8")


def safe_isodate() -> str:
    """Today's date, ISO, UTC."""
    import datetime as dt
    return dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d")


def record_phase(phase: str, evidence_dir: Path, vector_serial: str) -> int:
    """Run hardware/record_barriered.py once for one phase; capture its stdout to <phase>.jsonl."""
    assert phase in VALID_PHASES, f"unknown phase {phase!r}"
    assert evidence_dir.is_dir(), f"{evidence_dir} is not a directory"
    out = evidence_dir / f"{phase}.jsonl"
    env = dict(os.environ)
    env["PHASES"] = phase
    env["VECTOR_SERIAL"] = vector_serial
    env["NOTE"] = f"contribution recording, phase={phase}"
    print(f"\n== recording phase {phase} -> {out.relative_to(ROOT)}")
    with out.open("wb") as fh:
        rc = subprocess.call([sys.executable, str(RECORDER)], stdin=subprocess.DEVNULL, stdout=fh, env=env)
    if rc != 0:
        print(f"  recorder exited {rc}", file=sys.stderr)
    return rc


def parse_args(argv: list[str]):
    """Parse and validate argv. Returns (args, phases)."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--unit-handle", required=True, help="short handle, e.g. jo-1")
    parser.add_argument("--vector-serial", required=True, help="eight hex digits")
    parser.add_argument("--phases", default="rest,headlift,spins,straights,actions",
                        help="comma-separated phases; see hardware/record_barriered.py")
    parser.add_argument("--hardware-generation", choices=["1.0", "2.0"], default=None,
                        help="if omitted, you will be prompted")
    args = parser.parse_args(argv)
    assert HANDLE_RX.match(args.unit_handle), f"--unit-handle must match {HANDLE_RX.pattern}"
    assert SERIAL_RX.match(args.vector_serial), "--vector-serial must be 8 hex digits"
    phases = [p.strip() for p in args.phases.split(",") if p.strip()]
    assert 0 < len(phases) <= MAX_PHASES, f"phases must be 1..{MAX_PHASES}"
    for p in phases:
        assert p in VALID_PHASES, f"unknown phase {p!r}; one of {sorted(VALID_PHASES)}"
    return args, phases


def collect_metadata(unit_dir: Path, args) -> None:
    """Prompt for every field we cannot read off argv, then write metadata.yaml."""
    assert unit_dir.is_dir(), f"{unit_dir} does not exist"
    assert args.unit_handle, "args.unit_handle required"
    hardware = args.hardware_generation or prompt("hardware_generation (1.0 or 2.0)", "2.0")
    firmware = prompt("firmware_version (e.g. 1.9.0)", "unknown")
    mods = prompt("body_modifications (or 'stock')", "stock")
    contributor = prompt("contributor (your name or handle; blank to skip)")
    surface = prompt("surface (e.g. laminate)", "")
    write_metadata(unit_dir, hardware, firmware, mods, "CC BY 4.0",
                   "I release this data under CC BY 4.0.", contributor, surface)
    print(f"wrote {unit_dir.relative_to(ROOT)}/metadata.yaml")


def main(argv: list[str]) -> int:
    try:
        args, phases = parse_args(argv)
    except AssertionError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    unit_dir = UNITS / args.unit_handle
    if unit_dir.exists():
        print(f"{unit_dir.relative_to(ROOT)} already exists; pick a new handle or remove it first",
              file=sys.stderr)
        return 2
    (unit_dir / "evidence").mkdir(parents=True)
    collect_metadata(unit_dir, args)
    for phase in phases:
        if record_phase(phase, unit_dir / "evidence", args.vector_serial) != 0:
            print(f"recording failed on phase {phase}; stopping", file=sys.stderr)
            return 1
    print("\n== running the scrub")
    if subprocess.call([sys.executable, str(KIT / "scrub.py"), str(unit_dir),
                        "--serial", args.vector_serial]) != 0:
        return 1
    print("\n== running the validator")
    if subprocess.call([sys.executable, str(ROOT / "contrib" / "validator.py"), str(unit_dir)]) != 0:
        return 1
    print(f"\nready to commit {unit_dir.relative_to(ROOT)} and open a pull request.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
