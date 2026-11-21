"""Validate a contribution under contrib/units/<handle>-<n>/.

Checks the folder layout, parses metadata.yaml against a small schema, scans
every file for the identifiers the privacy scrub removes, and spot-checks the
recording JSONL for shape. The check runs locally and in the GitHub Action
.github/workflows/validate-contrib.yml; it uses only the Python standard
library so a contributor can run it without extra installs.

Exit codes: 0 valid, 1 one or more failures (listed on stderr), 2 bad arguments.

Usage:
    python3 contrib/validator.py contrib/units/<handle>-<n>
    python3 contrib/validator.py --all     # every unit under contrib/units/
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

MIN_UNIT_ROWS = 50               # a recording shorter than this is almost certainly a mistake
MAX_FILE_BYTES = 32_000_000
MIN_HANDLE_LEN, MAX_HANDLE_LEN = 3, 32
HANDLE_RX = re.compile(r"^[a-z0-9][a-z0-9-]{%d,%d}-[0-9]+$" % (MIN_HANDLE_LEN - 1, MAX_HANDLE_LEN - 3))

REQUIRED_META = {"hardware_generation", "firmware_version", "body_modifications",
                 "recorded_at", "licence", "sign_off"}
OPTIONAL_META = {"contributor", "surface", "room_temperature_c", "notes"}
ALLOWED_META = REQUIRED_META | OPTIONAL_META
ALLOWED_HARDWARE = {"1.0", "2.0"}
ALLOWED_LICENCES = {"CC BY 4.0", "CC0-1.0", "CC-BY-4.0", "CC-BY-SA-4.0"}

# Sign-off must mention the licence; the exact form is tolerant.
SIGN_OFF_RX = re.compile(r"I (release|hereby release) this data under (CC[ -]BY[ -]?4\.0|CC0[ -]1\.0|CC[ -]BY[ -]SA[ -]?4\.0)\.?", re.I)

# Patterns the privacy scrub would reduce. If any match remains, the submission failed the scrub.
LEAK_PATTERNS = (
    ("an IPv4 address", re.compile(r"\b(?!127\.0\.0\.1\b)(?!192\.0\.2\.)\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\b")),
    ("a Bluetooth address", re.compile(r"\b(?!02:00:00:00:00:01\b)(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}\b")),
    ("a GUID", re.compile(r"\b(?!00000000-)[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}\b")),
    ("an email address", re.compile(r"[A-Za-z0-9._-]+@(?!example\.invalid\b)[A-Za-z0-9-]+\.[a-z]{2,}")),
)
STATE_ROW_KEYS = {"gyro", "accel", "wl", "wr", "head", "lift", "pose"}


def load_yaml(path: Path) -> dict:
    """Parse a flat metadata.yaml without pyyaml: 'key: value', strings may be quoted or bare."""
    assert path.is_file(), f"{path} is not a file"
    text = path.read_text(encoding="utf-8")
    out: dict = {}
    for n, line in enumerate(text.splitlines(), 1):
        s = line.split("#", 1)[0].rstrip()
        if not s.strip():
            continue
        if ":" not in s:
            raise ValueError(f"metadata.yaml line {n}: no colon in {line!r}")
        k, _, v = s.partition(":")
        key, val = k.strip(), v.strip()
        if val and val[0] == val[-1] and val[0] in "\"'":
            val = val[1:-1]
        elif val.lower() in ("null", "~", ""):
            val = None
        elif val.lstrip("-").isdigit():
            val = int(val)
        out[key] = val
    return out


def check_metadata(path: Path, problems: list[str]) -> dict | None:
    """Parse and schema-check metadata.yaml. Returns the dict if valid, else None."""
    assert path.is_file(), f"{path} is not a file"
    try:
        meta = load_yaml(path)
    except ValueError as exc:
        problems.append(f"metadata.yaml: {exc}")
        return None
    missing = REQUIRED_META - meta.keys()
    extra = meta.keys() - ALLOWED_META
    if missing:
        problems.append(f"metadata.yaml: missing required fields {sorted(missing)}")
    if extra:
        problems.append(f"metadata.yaml: unknown fields {sorted(extra)}")
    hw = str(meta.get("hardware_generation", "")).strip()
    if hw not in ALLOWED_HARDWARE:
        problems.append(f"metadata.yaml: hardware_generation {hw!r} not in {sorted(ALLOWED_HARDWARE)}")
    lic = str(meta.get("licence", "")).strip()
    if lic and lic not in ALLOWED_LICENCES:
        problems.append(f"metadata.yaml: licence {lic!r} not in {sorted(ALLOWED_LICENCES)}")
    sign = str(meta.get("sign_off", "")).strip()
    if not SIGN_OFF_RX.match(sign):
        problems.append(f"metadata.yaml: sign_off {sign!r} does not match the required form")
    return meta


def scan_leaks(root: Path, problems: list[str]) -> None:
    """Report every file that still carries a pattern the scrub should have removed."""
    assert root.is_dir(), f"{root} is not a directory"
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path.is_symlink():
            continue
        if path.stat().st_size > MAX_FILE_BYTES:
            problems.append(f"{path.relative_to(root)}: larger than {MAX_FILE_BYTES} bytes")
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            problems.append(f"{path.relative_to(root)}: not valid UTF-8; keep text files only")
            continue
        for name, rx in LEAK_PATTERNS:
            hits = rx.findall(text)
            if hits:
                problems.append(f"{path.relative_to(root)}: carries {name} ({len(hits)} occurrence(s))")


def check_recording(path: Path, problems: list[str]) -> None:
    """Sanity check for one *.jsonl recording: valid lines, enough state rows, known keys."""
    assert path.is_file(), f"{path} is not a file"
    rows = 0
    state_rows = 0
    for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError as exc:
            problems.append(f"{path.name}: line {n} is not valid JSON ({exc.msg})")
            return
        rows += 1
        if obj.get("k", "s") == "s" or STATE_ROW_KEYS & obj.keys():
            state_rows += 1
    if rows == 0:
        problems.append(f"{path.name}: empty recording")
    elif state_rows < MIN_UNIT_ROWS:
        problems.append(f"{path.name}: only {state_rows} state row(s); min {MIN_UNIT_ROWS}")


def validate(unit: Path) -> list[str]:
    """Run every check on one unit folder. Returns the list of problems (empty = valid)."""
    assert unit.is_dir(), f"{unit} is not a directory"
    problems: list[str] = []
    if not HANDLE_RX.match(unit.name):
        problems.append(f"{unit.name}: folder name must match {HANDLE_RX.pattern}")
    meta_path = unit / "metadata.yaml"
    if not meta_path.is_file():
        problems.append("metadata.yaml is missing")
    else:
        check_metadata(meta_path, problems)
    ev = unit / "evidence"
    if not ev.is_dir():
        problems.append("evidence/ is missing")
    else:
        recs = sorted(ev.glob("*.jsonl"))
        if not recs:
            problems.append("evidence/ holds no *.jsonl recording")
        for r in recs:
            check_recording(r, problems)
    scan_leaks(unit, problems)
    return problems


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("unit", nargs="?", help="path to one contrib/units/<handle>-<n>")
    parser.add_argument("--all", action="store_true", help="validate every unit under contrib/units/")
    args = parser.parse_args(argv)
    if args.all:
        root = Path(__file__).resolve().parent / "units"
        if not root.is_dir():
            print("no contrib/units/ directory; nothing to do")
            return 0
        units = [p for p in sorted(root.iterdir()) if p.is_dir()]
    elif args.unit:
        units = [Path(args.unit)]
    else:
        parser.error("pass a unit folder or --all")
        return 2
    any_fail = False
    for u in units:
        print(f"== {u}")
        problems = validate(u)
        for p in problems:
            print(f"  FAIL  {p}", file=sys.stderr)
        if problems:
            any_fail = True
        else:
            print("  ok")
    return 1 if any_fail else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
