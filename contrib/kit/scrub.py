"""Privacy scrub for a contribution folder.

Walks a folder produced by contrib/kit/record.py (or by running the recorder
manually) and strips the identifiers that must not reach the public repository:
robot serial numbers, IP addresses, Bluetooth addresses, SDK session GUIDs and
email addresses. Each is replaced with a stable fictional value or redacted.

Exit codes: 0 clean, 1 refused (one or more identifiers could not be reduced),
2 bad arguments.

Usage:
    python3 contrib/kit/scrub.py <folder> [--serial <hex>]
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

MAX_FILE_BYTES = 32_000_000
STUB_SERIAL = "deadbeef"
STUB_IP = "192.0.2.10"           # RFC 5737 TEST-NET-1
STUB_MAC = "02:00:00:00:00:01"   # locally administered
STUB_GUID = "00000000-0000-0000-0000-000000000000"

# Regex patterns for things that must be scrubbed.
IP_RX = re.compile(r"\b(?!127\.0\.0\.1\b)(?!192\.0\.2\.)\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\b")
MAC_RX = re.compile(r"\b(?!02:00:00:00:00:01\b)(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}\b")
GUID_RX = re.compile(r"\b(?!00000000-)[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}\b")
EMAIL_RX = re.compile(r"[A-Za-z0-9._-]+@[A-Za-z0-9-]+\.[a-z]{2,}")
SERIAL_RX = re.compile(r"\b[0-9a-f]{8}\b")  # eight hex digits; conservative match

# Patterns for binary content in otherwise text-ish files: refuse rather than guess.
BINARY_HINTS = (b"\x00", b"\xff\xd8\xff")  # NULs, JPEG start marker


def scrub_text(text: str, serial: str | None) -> tuple[str, int]:
    """Apply every scrub to one text blob. Returns (text, number of changes)."""
    assert isinstance(text, str), "text must be a string"
    assert serial is None or re.fullmatch(r"[0-9a-fA-F]{8}", serial), "serial is 8 hex digits"
    n = 0
    if serial:
        lower = serial.lower()
        m = text.count(lower)
        text = text.replace(lower, STUB_SERIAL)
        n += m
        upper = serial.upper()
        if upper != lower:
            m = text.count(upper)
            text = text.replace(upper, STUB_SERIAL)
            n += m
    text, m = IP_RX.subn(STUB_IP, text)
    n += m
    text, m = MAC_RX.subn(STUB_MAC, text)
    n += m
    text, m = GUID_RX.subn(STUB_GUID, text)
    n += m
    text, m = EMAIL_RX.subn("contributor@example.invalid", text)
    n += m
    return text, n


def is_text_file(path: Path) -> bool:
    """Heuristic: a file is text if a probe of the first 4 KiB has no NUL and no JPEG marker."""
    assert path.is_file(), f"{path} is not a file"
    probe = path.read_bytes()[:4096]
    return not any(h in probe for h in BINARY_HINTS)


def scrub_tree(root: Path, serial: str | None) -> int:
    """Scrub every text file under root; return the count of non-clean files."""
    assert root.is_dir(), f"{root} is not a directory"
    bad = 0
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path.is_symlink():
            continue
        if path.stat().st_size > MAX_FILE_BYTES:
            print(f"SKIP {path.relative_to(root)}: larger than {MAX_FILE_BYTES} bytes", file=sys.stderr)
            bad += 1
            continue
        if not is_text_file(path):
            print(f"REFUSE {path.relative_to(root)}: non-text file; remove it or review by hand", file=sys.stderr)
            bad += 1
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            print(f"REFUSE {path.relative_to(root)}: not valid UTF-8", file=sys.stderr)
            bad += 1
            continue
        new, n = scrub_text(text, serial)
        if new != text:
            path.write_text(new, encoding="utf-8")
            print(f"scrubbed {path.relative_to(root)} ({n} changes)")
    return bad


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", help="path to the submission folder")
    parser.add_argument("--serial", default=None,
                        help="the robot's own 8-hex-digit serial to replace (optional)")
    args = parser.parse_args(argv)
    root = Path(args.folder)
    if not root.is_dir():
        print(f"{root} is not a directory", file=sys.stderr)
        return 2
    assert args.serial is None or len(args.serial) == 8, "--serial is 8 hex digits"
    bad = scrub_tree(root, args.serial)
    if bad:
        print(f"{bad} file(s) could not be reduced; see messages above", file=sys.stderr)
        return 1
    print("scrub: clean")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
