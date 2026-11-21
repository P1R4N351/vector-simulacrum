#!/usr/bin/env python3
"""The Vector simulacrum as an MCP server over stdio..

    <python> <skill>/mcp_server.py        any MCP client that starts a stdio server

Read-only and robot-free: see simulacrum/mcp_service.py for the tools and
what each may read. Needs Python 3.9 or later and nothing else (no MuJoCo,
no anki_vector, no MCP library), on Linux, macOS or Windows. One JSON-RPC
message per line in, one per line out, UTF-8 with a bare newline whatever
the platform's own text mode would do; diagnostics go to stderr with a UTC
time stamp.

Exit codes: 0 stdin closed, 3 the message bound was reached or Python is
older than 3.9.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

MIN_PYTHON = (3, 9)
if sys.version_info < MIN_PYTHON:
    sys.stderr.write("vector-simulacrum-mcp needs Python %d.%d or later; this is %s\n"
                     % (MIN_PYTHON + (sys.version.split()[0],)))
    sys.exit(3)

sys.path.insert(0, str(Path(__file__).resolve().parent))

from simulacrum import mcp_service  # noqa: E402

MAX_MESSAGES = 1_000_000
MAX_LINE_CHARS = 1_000_000


def log(text: str) -> None:
    stamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    sys.stderr.write(f"{stamp} vector-simulacrum-mcp {text}\n")
    sys.stderr.flush()


def answer(line: str) -> None:
    """Parse one line, dispatch it, write the response if there is one."""
    if len(line) > MAX_LINE_CHARS:
        reply = {"jsonrpc": "2.0", "id": None, "error": {"code": -32600, "message": "message too long"}}
    else:
        try:
            msg = json.loads(line)
        except json.JSONDecodeError as exc:
            msg, reply = None, {"jsonrpc": "2.0", "id": None, "error": {"code": -32700, "message": f"parse error: {exc}"}}
        if isinstance(msg, dict):
            reply = mcp_service.dispatch(msg)
        elif msg is not None:
            reply = {"jsonrpc": "2.0", "id": None, "error": {"code": -32600, "message": "not a JSON-RPC object"}}
    if reply is not None:
        sys.stdout.write(json.dumps(reply) + "\n")
        sys.stdout.flush()


def use_plain_utf8() -> None:
    """MCP over stdio is UTF-8 with a bare newline; Windows text mode would write CRLF in a local code page."""
    for stream in (sys.stdin, sys.stdout, sys.stderr):
        stream.reconfigure(encoding="utf-8", newline="\n")
    assert sys.stdout.encoding.lower().replace("-", "") == "utf8", "stdout is not UTF-8"


def main() -> int:
    use_plain_utf8()
    log(f"started, root {mcp_service.SKILL_ROOT}, Python {sys.version.split()[0]} on {sys.platform}")
    for count, line in enumerate(sys.stdin):
        if count >= MAX_MESSAGES:
            log(f"stopping after {MAX_MESSAGES} messages")
            return 3
        if line.strip():
            answer(line)
    log("stdin closed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
