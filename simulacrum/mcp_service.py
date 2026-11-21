"""The simulacrum's MCP tools: read the table and the recordings, plan a recording. Authored-by: CPCS.

Transport is in mcp_server.py; this module holds the tools and the JSON-RPC
dispatch, so tests can call them without a pipe.

READ-ONLY, AND NO ROBOT. Nothing here imports anki_vector, opens a socket,
or runs the recorder. recorder_plan writes out the command a person runs;
it does not run it. No tool writes a file.

Files a tool reads must be .jsonl files inside the skill directory, no
larger than MAX_FILE_BYTES. A recording placed elsewhere is refused rather
than followed.

Tools are added as the work needs them; tools/list advertises exactly the
tools in TOOLS, and each one is exercised in tests/test_mcp.py.
"""

from __future__ import annotations

import ast
import json
import math
import re
import statistics
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Any, Callable, Dict, List, Optional, Tuple

from . import params, recording, wheelstand

SERVER_NAME = "vector-simulacrum"
SERVER_VERSION = "0.1.0"
DEFAULT_PROTOCOL = "2025-06-18"
SKILL_ROOT = Path(__file__).resolve().parent.parent
RECORDER = SKILL_ROOT / "hardware" / "record_barriered.py"
MAX_FILE_BYTES = 64 * 1024 * 1024
MAX_LIST = 200
SOURCE_CHARS = 400
NOTE_CHARS = 400
NOTE_ALLOWED = re.compile(r"^[A-Za-z0-9 ,.;:()/+=_-]*$")  # nothing a shell on any platform reads specially inside quotes
OUT_NAME = re.compile(r"^[a-z0-9][a-z0-9._-]{0,60}\.jsonl$")
NOT_COMPUTABLE = (AssertionError, KeyError, ValueError, ZeroDivisionError, statistics.StatisticsError)

# What each recorder phase needs around the robot, from the recorder's own docstring.
# tests/test_mcp.py checks the keys against the recorder's KNOWN_PHASES.
PHASE_SPACE = {
    "rest": ("no travel", False),
    "headlift": ("no travel", False),
    "spins": ("turns on the spot; allow 100 mm all round", False),
    "straights": ("up to 450 mm forward of the start", True),
    "reversal": ("about 120 mm forward", True),
    "arcs": ("a circle of about 150 mm radius", True),
    "actions": ("about 120 mm forward", True),
    "repeat": ("about 30 mm; must be last", False),
    "edge": ("drives to an edge until the robot's own cliff reaction stops it; needs EDGE_START_MM by ruler", True),
    "wheelstand": ("no travel; head level and lift down, then nothing commanded while a person tips the robot; "
                   "a soft surface", False),
    "wheelie": ("drives to its light cube and tips onto its rear (the robot's own pop_a_wheelie); the cube within "
                "reach and the room lit, since it finds the cube with its camera; must be last", True),
    "nudge": ("short moves back and forth at 40 mm/s, under 25 mm each way, to seat a robot on its charger "
              "ramp; stops on charger contact", True),
    "liftspins": ("spins on the spot at 150 mm/s both ways with the lift down, then up, then down again; "
                  "allow 100 mm all round", True),
    "endurance": ("sustained load: spins on the spot both ways at ENDURANCE_MMPS (default 300, above the treads' top "
                  "speed) for ENDURANCE_S (default 600); allow 100 mm all round; drains the battery, stops at 3.6 V", True),
    "dock": ("drives to its charger (the robot's own drive_on_charger), which it finds with its camera; must be "
             "last, not with wheelie; alone on the charger it commands nothing", True),
}


class ToolError(Exception):
    """A refusal or a bad argument, reported to the caller as an error result."""


def _readable(path_text: Optional[str], root: Path, default: Path) -> Path:
    """The file a tool may read: a .jsonl inside root, present, and not too large."""
    path = default if not path_text else Path(path_text)
    if not path.is_absolute():
        path = root / path
    resolved = path.resolve()
    if root.resolve() not in resolved.parents:
        raise ToolError(f"refused: {resolved} is outside {root.resolve()}")
    if resolved.suffix != ".jsonl":
        raise ToolError(f"refused: {resolved.name} is not a .jsonl file")
    if not resolved.is_file():
        raise ToolError(f"no such file: {resolved}")
    if resolved.stat().st_size > MAX_FILE_BYTES:
        raise ToolError(f"refused: {resolved.name} is larger than {MAX_FILE_BYTES} bytes")
    return resolved


def _attempt(label: str, call: Callable[[], Any]) -> Any:
    """One measurement; a recording that cannot support it gives a reason instead of a figure."""
    try:
        return call()
    except NOT_COMPUTABLE as exc:
        return {"not_computable": f"{label}: {type(exc).__name__}: {str(exc)[:200]}"}


def tool_parameters(args: Dict[str, Any]) -> Dict[str, Any]:
    """Entries of the parameter table for one model, filtered by provenance and by part of the name."""
    model = str(args.get("model") or "vector_1")
    if model not in params.MODELS:
        raise ToolError(f"model must be one of {sorted(params.MODELS)}")
    doc = params.load(model=model)
    provenance = args.get("provenance")
    if provenance is not None and provenance not in params.PROVENANCE:
        raise ToolError(f"provenance must be one of {list(params.PROVENANCE)}")
    part = str(args.get("name") or "")
    limit = int(args.get("limit") or MAX_LIST)
    if not 1 <= limit <= MAX_LIST:
        raise ToolError(f"limit must be 1..{MAX_LIST}")
    found = []
    for name in sorted(doc["parameters"]):
        entry = doc["parameters"][name]
        if (provenance is None or entry["provenance"] == provenance) and part in name:
            row = {key: entry[key] for key in ("value", "unit", "provenance", "kind", "low", "high", "inherited",
                                               "overrides_base") if key in entry}
            row["name"], row["source"] = name, str(entry.get("source", ""))[:SOURCE_CHARS]
            found.append(row)
    return {"model": model, "robot": doc["robot"], "counts": params.summary(doc), "matched": len(found),
            "entries": found[:limit]}


def _run_markers(commands: List[Dict[str, Any]]) -> Dict[str, Any]:
    """What the recorder said about the run: its begin and end rows, and anything that ended or skipped motion."""
    first = {row["name"]: row for row in reversed(commands)}
    keep = ("phases", "control", "on_charger", "cliff", "note", "abort")
    trouble = sorted({row["name"] for row in commands
                      if row["name"] in ("skip", "no_control", "abort_watch", "lock_held") or row["name"].endswith("_failed")})
    return {"begin": {k: v for k, v in first.get("begin", {}).items() if k in keep},
            "end": {k: v for k, v in first.get("end", {}).items() if k in keep}, "trouble": trouble}


def _spins_table(states: List[Dict[str, Any]], commands: List[Dict[str, Any]], spacing: float) -> List[Dict[str, Any]]:
    ratios = recording.spin_ratios(states, commands, spacing)
    wheels = {seg.label: seg for seg in recording._wheel_segments(commands)}
    out = []
    for label, figures in sorted(ratios.items()):
        seg = wheels[label]
        out.append({"label": label, "wheel_mmps": abs(seg.left), "n": figures["n"], "head_deg": round(figures["head_deg"], 2),
                    "whole": round(figures["whole"], 4), "gyro_z": round(figures["gyro_z"], 4)})
    return out


def summarise(path: Path) -> Dict[str, Any]:
    """Everything the recording module can say about one recording file."""
    states, commands = recording.load(path)
    v = params.values(params.load())
    cliff_rows = sum(1 for row in states if int(row.get("status", 0)) & recording.STATUS_CLIFF)
    return {"file": str(path), "sha256": recording.sha256(path), "state_rows": len(states),
            "command_rows": len(commands), "markers": _run_markers(commands), "cliff_rows": cliff_rows,
            "state_interval_ms": _attempt("state interval", lambda: recording.state_interval_ms(states)),
            "spins": _attempt("spins", lambda: _spins_table(states, commands, v["tread_spacing_mm"])),
            "scatter_in_spins": _attempt("scatter", lambda: recording.moving_scatter(states, commands, v["gravity_mmps2"])),
            "gyro_frame": _attempt("gyro frame", lambda: recording.gyro_frame(states)),
            "gyro_at_rest": _attempt("gyro at rest", lambda: recording.gyro_at_rest(states))}


def tool_recording_summary(args: Dict[str, Any], root: Path = SKILL_ROOT) -> Dict[str, Any]:
    return summarise(_readable(args.get("path"), root, recording.RECORDING))


def _by_speed(spins: List[Dict[str, Any]]) -> Dict[str, float]:
    speeds: Dict[float, List[float]] = {}
    for spin in spins:
        speeds.setdefault(spin["wheel_mmps"], []).append(spin["whole"])
    return {f"{speed:g}": round(statistics.mean(values), 4) for speed, values in sorted(speeds.items())}


def compare_spins(path: Path) -> Dict[str, Any]:
    """Turn rate over the no-slip rate, per spin and by wheel speed, beside the first recording and the table."""
    v = params.values(params.load())
    states, commands = recording.load(path)
    spins = _spins_table(states, commands, v["tread_spacing_mm"])
    first_states, first_commands = recording.load(recording.RECORDING)
    first = _spins_table(first_states, first_commands, v["tread_spacing_mm"])
    out = {"file": str(path), "spins": spins, "by_wheel_speed": _by_speed(spins),
           "mean_whole": round(statistics.mean(s["whole"] for s in spins), 4),
           "first_recording_by_wheel_speed": _by_speed(first),
           "table_reference": v["spin_rate_fraction_measured"],
           "note": "whole = gyro x and z together over (|left| + |right|) / tread spacing, second half of each spin."}
    assert all(math.isfinite(s["whole"]) for s in spins), "a spin figure is not a number"
    return out


def tool_spin_fractions(args: Dict[str, Any], root: Path = SKILL_ROOT) -> Dict[str, Any]:
    return compare_spins(_readable(args.get("path"), root, recording.RECORDING))


def tool_wheelstand_balance(args: Dict[str, Any], root: Path = SKILL_ROOT) -> Dict[str, Any]:
    """Centre of mass from a wheelstand recording; with no path, the balance angles the table predicts."""
    width = args.get("outer_tread_width_mm")
    width = float(width) if width is not None else None
    if width is not None and not 40.0 <= width <= 80.0:
        raise ToolError("outer_tread_width_mm must be 40..80 mm (a ruler across the outside of both treads)")
    v = params.values(params.load())
    if not args.get("path"):
        g = wheelstand.geometry(v, width)
        return {"expected_deg": wheelstand.expected_deg(v["com_x_mm"], v["com_z_mm"], g), "geometry_mm": g,
                "table": {"com_x_mm": v["com_x_mm"], "com_z_mm": v["com_z_mm"]},
                "note": "the angles the table's assumed centre of mass would balance at; give path for a recording"}
    path = _readable(args.get("path"), root, recording.RECORDING)
    out = wheelstand.describe(path, width)
    out["file"] = str(path)
    return out


def known_phases() -> Tuple[str, ...]:
    """The recorder's KNOWN_PHASES, read from its source without importing it (it imports anki_vector)."""
    tree = ast.parse(RECORDER.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, "id", "") == "KNOWN_PHASES" for t in node.targets):
            phases = ast.literal_eval(node.value)
            assert isinstance(phases, tuple) and phases, "KNOWN_PHASES is not a tuple"
            return phases
    raise AssertionError("KNOWN_PHASES not found in the recorder")


def _plan_problems(phases: List[str], note: str, edge_start: Optional[float], out_name: str) -> List[str]:
    known = known_phases()
    problems = [f"unknown phase {p!r}" for p in phases if p not in known]
    if not phases:
        problems.append("no phases chosen")
    if "repeat" in phases and phases[-1] != "repeat":
        problems.append("repeat must be the last phase")
    if "wheelie" in phases and (phases[-1] != "wheelie" or "repeat" in phases):
        problems.append("wheelie must be the last phase, and not with repeat")
    if "dock" in phases and (phases[-1] != "dock" or "wheelie" in phases):
        problems.append("dock must be the last phase, and not with wheelie")
    if len(set(phases)) != len(phases):
        problems.append("a phase is listed twice")
    if "edge" in phases and (edge_start is None or not 5.0 <= edge_start <= 30.0 * 3.0):
        problems.append("edge needs edge_start_mm, measured by ruler, 5..90 mm at the default 30 mm/s for 3 s")
    if len(note) > NOTE_CHARS or not NOTE_ALLOWED.match(note):
        problems.append("note: letters, digits, spaces and , . ; : ( ) / + = _ - only, at most 400 characters")
    if not OUT_NAME.match(out_name):
        problems.append("out_name: lower-case letters, digits, . _ - and ending in .jsonl")
    return problems


def remote_command(phases: List[str], note: str, edge_start: Optional[float]) -> str:
    """What the recorder's host runs, as one string for its POSIX shell.

    It holds no double quote, dollar sign, backtick or backslash (the note is
    checked for all of them), so the same string can sit inside double quotes
    on a POSIX shell, in Windows cmd.exe and in PowerShell, and ssh hands it to
    the remote shell unchanged. The note is in single quotes there.
    """
    words = ["env", f"PHASES={','.join(phases)}", f"NOTE='{note}'"]
    if edge_start is not None:
        words.append(f"EDGE_START_MM={edge_start:g}")
    remote = " ".join(words + ["python3", "-"])
    assert not set(remote) & set('"$`\\'), "the remote command would not survive the local shell"
    return remote


def tool_recorder_plan(args: Dict[str, Any]) -> Dict[str, Any]:
    """The command for a person to run the stored recorder. Runs nothing."""
    phases = [str(p).strip() for p in (args.get("phases") or []) if str(p).strip()]
    note = str(args.get("note") or "")
    edge = args.get("edge_start_mm")
    edge_start = float(edge) if edge is not None else None
    out_name = str(args.get("out_name") or "run2.jsonl")
    problems = _plan_problems(phases, note, edge_start, out_name)
    if problems:
        raise ToolError("; ".join(problems))
    remote = remote_command(phases, note, edge_start)
    stdin = PurePosixPath("hardware", RECORDER.name)
    return {"runs_nothing": True, "run_from": str(SKILL_ROOT),
            "argv": ["ssh", "WIREPOD_HOST", remote], "stdin_file": str(stdin), "stdout_file": out_name,
            "shell": {"posix_sh": f'ssh WIREPOD_HOST "{remote}" < {stdin} > {out_name}',
                      "windows_cmd": f'ssh WIREPOD_HOST "{remote}" < {PureWindowsPath(stdin)} > {out_name}'},
            "replace": "WIREPOD_HOST: the host that ran the first recording",
            "recorder_sha256": recording.sha256(RECORDER),
            "space": {p: PHASE_SPACE[p][0] for p in phases},
            "watch_it_drive": [p for p in phases if PHASE_SPACE[p][1]],
            "always": "a wall or a soft landing on every side; never an open-edged table"}


def _schema(properties: Dict[str, Any], required: Tuple[str, ...] = ()) -> Dict[str, Any]:
    return {"type": "object", "properties": properties, "required": list(required), "additionalProperties": False}


PATH_ARG = {"type": "string", "description": ".jsonl recording inside the skill directory; default: the first recording"}
TOOLS: Dict[str, Tuple[str, Dict[str, Any], Callable[[Dict[str, Any]], Dict[str, Any]]]] = {
    "parameters": ("Entries of the Vector parameter table with provenance, range and source, for Vector 1.0 "
                   "(vector_1, default) or 2.0 (vector_2, where inherited 1.0 measurements are marked).",
                   _schema({"model": {"type": "string", "enum": sorted(params.MODELS)},
                            "provenance": {"type": "string", "enum": list(params.PROVENANCE)},
                            "name": {"type": "string", "description": "part of the parameter name"},
                            "limit": {"type": "integer", "minimum": 1, "maximum": MAX_LIST}}), tool_parameters),
    "recording_summary": ("Rows, run markers, spins, sensor scatter and gyro figures of one recording of the real robot.",
                          _schema({"path": PATH_ARG}), tool_recording_summary),
    "spin_fractions": ("Turn rate over the no-slip rate for each spin in a recording, against the first recording "
                       "and the table.", _schema({"path": PATH_ARG}), tool_spin_fractions),
    "wheelstand_balance": ("Centre of mass (com_x, com_z) from a wheelstand recording: balance angles on the rear wheels "
                           "and on one tread edge, read by the robot's own accelerometer while a person tips it. "
                           "With no path, the angles the table's centre of mass predicts.",
                           _schema({"path": {"type": "string", "description": ".jsonl wheelstand recording inside the skill "
                                                                       "directory"},
                                    "outer_tread_width_mm": {"type": "number", "description":
                                                             "ruler reading across the outside of both treads"}}),
                           tool_wheelstand_balance),
    "recorder_plan": ("Checks a choice of recorder phases and writes out the command for a person to run. Runs nothing "
                      "and never connects to the robot.",
                      _schema({"phases": {"type": "array", "items": {"type": "string"}},
                               "note": {"type": "string"}, "edge_start_mm": {"type": "number"},
                               "out_name": {"type": "string"}}, ("phases",)), tool_recorder_plan),
}


def _result(payload: Any, is_error: bool) -> Dict[str, Any]:
    return {"content": [{"type": "text", "text": json.dumps(payload, indent=1, default=str)}], "isError": is_error}


def call_tool(name: Any, args: Any) -> Dict[str, Any]:
    if name not in TOOLS:
        return _result({"error": f"unknown tool {name!r}", "known": sorted(TOOLS)}, True)
    try:
        return _result(TOOLS[name][2](args if isinstance(args, dict) else {}), False)
    except ToolError as exc:
        return _result({"error": str(exc)}, True)
    except NOT_COMPUTABLE as exc:
        return _result({"error": f"{name}: {type(exc).__name__}: {str(exc)[:300]}"}, True)


def dispatch(msg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """One JSON-RPC message in, one response out, or None for a notification."""
    method, msg_id = msg.get("method"), msg.get("id")
    body = msg.get("params") if isinstance(msg.get("params"), dict) else {}
    if method == "initialize":
        return {"jsonrpc": "2.0", "id": msg_id, "result": {
            "protocolVersion": body.get("protocolVersion") or DEFAULT_PROTOCOL, "capabilities": {"tools": {}},
            "serverInfo": {"name": SERVER_NAME, "version": SERVER_VERSION}}}
    if method == "ping":
        return {"jsonrpc": "2.0", "id": msg_id, "result": {}}
    if method == "tools/list":
        tools = [{"name": n, "description": d, "inputSchema": s} for n, (d, s, _) in TOOLS.items()]
        return {"jsonrpc": "2.0", "id": msg_id, "result": {"tools": tools}}
    if method == "tools/call":
        return {"jsonrpc": "2.0", "id": msg_id, "result": call_tool(body.get("name"), body.get("arguments"))}
    if msg_id is None:
        return None
    return {"jsonrpc": "2.0", "id": msg_id, "error": {"code": -32601, "message": f"method not found: {method}"}}
