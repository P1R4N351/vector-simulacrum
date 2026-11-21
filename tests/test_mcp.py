"""The MCP server: protocol, the four tools, and what they refuse. Needs no MuJoCo.."""

from __future__ import annotations

import json
import math
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any, Dict, List

import _common  # noqa: F401  (puts the skill on the path)
import test_recorder
from simulacrum import mcp_service, params, recording

SERVER = _common.ROOT / "mcp_server.py"


def call(name: str, args: Dict[str, Any]) -> Dict[str, Any]:
    """A tool call through the dispatcher, as a client sees it: (error flag, decoded payload)."""
    reply = mcp_service.dispatch({"jsonrpc": "2.0", "id": 9, "method": "tools/call",
                                  "params": {"name": name, "arguments": args}})
    assert reply is not None and reply["id"] == 9, "no reply to a request"
    result = reply["result"]
    return {"error": result["isError"], "body": json.loads(result["content"][0]["text"])}


class Protocol(unittest.TestCase):
    def test_initialize_echoes_the_clients_protocol_version(self) -> None:
        reply = mcp_service.dispatch({"jsonrpc": "2.0", "id": 1, "method": "initialize",
                                      "params": {"protocolVersion": "2025-03-26"}})
        self.assertEqual(reply["result"]["protocolVersion"], "2025-03-26")
        self.assertEqual(reply["result"]["serverInfo"]["name"], "vector-simulacrum")
        bare = mcp_service.dispatch({"jsonrpc": "2.0", "id": 2, "method": "initialize"})
        self.assertEqual(bare["result"]["protocolVersion"], mcp_service.DEFAULT_PROTOCOL)

    def test_a_notification_gets_no_reply_and_an_unknown_method_gets_an_error(self) -> None:
        self.assertIsNone(mcp_service.dispatch({"jsonrpc": "2.0", "method": "notifications/initialized"}))
        reply = mcp_service.dispatch({"jsonrpc": "2.0", "id": 3, "method": "resources/list"})
        self.assertEqual(reply["error"]["code"], -32601)

    def test_the_tool_list_is_exactly_the_wired_tools(self) -> None:
        reply = mcp_service.dispatch({"jsonrpc": "2.0", "id": 4, "method": "tools/list"})
        tools = reply["result"]["tools"]
        self.assertEqual([t["name"] for t in tools],
                         ["parameters", "recording_summary", "spin_fractions", "wheelstand_balance", "recorder_plan"])
        self.assertTrue(all(t["inputSchema"]["type"] == "object" and t["description"] for t in tools))
        unknown = call("drive_robot", {})
        self.assertTrue(unknown["error"])
        self.assertIn("unknown tool", unknown["body"]["error"])

    def test_the_server_answers_over_a_pipe(self) -> None:
        lines = [{"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2025-06-18"}},
                 {"jsonrpc": "2.0", "method": "notifications/initialized"},
                 {"jsonrpc": "2.0", "id": 2, "method": "tools/call",
                  "params": {"name": "parameters", "arguments": {"name": "com_x"}}}]
        text = "\n".join(json.dumps(line) for line in lines) + "\nnot json\n"
        done = subprocess.run([sys.executable, str(SERVER)], input=text.encode("utf-8"), capture_output=True, timeout=60)
        self.assertEqual(done.returncode, 0, done.stderr[-400:])
        self.assertNotIn(b"\r", done.stdout)
        self.assertTrue(done.stdout.endswith(b"\n"))
        replies = [json.loads(line) for line in done.stdout.decode("utf-8").split("\n") if line]
        self.assertEqual([r.get("id") for r in replies], [1, 2, None])
        self.assertEqual(replies[2]["error"]["code"], -32700)
        body = json.loads(replies[1]["result"]["content"][0]["text"])
        self.assertEqual([e["name"] for e in body["entries"]], ["com_x_mm"])


class Parameters(unittest.TestCase):
    def test_filters_by_provenance_and_name(self) -> None:
        counts = params.summary(params.load())
        measured = call("parameters", {"provenance": "measured"})["body"]
        self.assertEqual(measured["matched"], counts["measured"])
        self.assertTrue(all(e["provenance"] == "measured" for e in measured["entries"]))
        com = call("parameters", {"name": "com_x"})["body"]["entries"][0]
        self.assertEqual(com["provenance"], "assumed")
        self.assertTrue(com["low"] < com["value"] < com["high"])
        self.assertTrue(call("parameters", {"provenance": "guessed"})["error"])

    def test_the_vector_2_set_marks_what_it_inherits(self) -> None:
        body = call("parameters", {"model": "vector_2", "name": "display_resolution"})["body"]
        self.assertEqual((body["model"], body["entries"][0]["value"], body["entries"][0]["overrides_base"]),
                         ("vector_2", [160, 80], True))
        noload = call("parameters", {"model": "vector_2", "name": "tread_noload"})["body"]["entries"][0]
        self.assertIn("not verified on a Vector 2.0", noload["inherited"])
        self.assertNotIn("inherited", call("parameters", {"name": "tread_noload"})["body"]["entries"][0])
        self.assertTrue(call("parameters", {"model": "vector_3"})["error"])


class Recordings(unittest.TestCase):
    def test_summary_of_the_first_recording(self) -> None:
        out = call("recording_summary", {})
        self.assertFalse(out["error"])
        body = out["body"]
        self.assertEqual((body["state_rows"], body["command_rows"]), (2700, 51))
        self.assertEqual(body["sha256"], recording.sha256(recording.RECORDING))
        self.assertEqual(len(body["spins"]), 6)
        self.assertEqual(body["gyro_frame"]["n"], 369)
        self.assertGreater(body["cliff_rows"], 0)

    def test_spin_fractions_of_the_first_recording_match_the_table(self) -> None:
        body = call("spin_fractions", {})["body"]
        self.assertAlmostEqual(body["mean_whole"], body["table_reference"], delta=0.005)
        self.assertEqual(sorted(body["by_wheel_speed"]), ["100", "30", "60"])
        self.assertEqual(body["by_wheel_speed"], body["first_recording_by_wheel_speed"])

    def test_spin_fractions_of_a_constructed_second_recording(self) -> None:
        truth, head = {150.0: 0.62, 220.0: 0.58}, 0.3
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "run2.jsonl"
            write_constructed_run(path, truth, head)
            body = mcp_service.tool_spin_fractions({"path": str(path)}, root=Path(tmp))
            summary = mcp_service.tool_recording_summary({"path": "run2.jsonl"}, root=Path(tmp))
        self.assertEqual(len(body["spins"]), 4)
        for spin in body["spins"]:
            self.assertAlmostEqual(spin["whole"], truth[spin["wheel_mmps"]], delta=1e-4)
            self.assertAlmostEqual(spin["gyro_z"], truth[spin["wheel_mmps"]] * math.cos(head), delta=1e-4)
        self.assertEqual(body["by_wheel_speed"], {"150": 0.62, "220": 0.58})
        self.assertEqual(summary["markers"]["begin"]["phases"], ["spins"])
        self.assertEqual(summary["markers"]["trouble"], [])
        self.assertIn("not_computable", summary["gyro_frame"])


class Wheelstand(unittest.TestCase):
    def test_with_no_recording_it_gives_the_tables_expected_angles(self) -> None:
        body = call("wheelstand_balance", {})["body"]
        self.assertAlmostEqual(body["expected_deg"]["rear"], 64.885, delta=0.001)
        self.assertAlmostEqual(body["expected_deg"]["side"], 51.843, delta=0.001)
        ruler = call("wheelstand_balance", {"outer_tread_width_mm": 60})["body"]
        self.assertEqual(ruler["geometry_mm"]["edge_y"], 30.0)
        self.assertTrue(call("wheelstand_balance", {"outer_tread_width_mm": 5})["error"])

    def test_a_constructed_session_through_the_tool(self) -> None:
        import test_wheelstand
        session, truth = test_wheelstand.a_session(-21.0, 24.0, 0.4)
        with tempfile.TemporaryDirectory() as tmp:
            session.write(Path(tmp) / "ws.jsonl")
            body = mcp_service.tool_wheelstand_balance({"path": "ws.jsonl", "outer_tread_width_mm": 56}, root=Path(tmp))
        self.assertEqual(body["stands"]["rear"]["method"], "bracket")
        self.assertAlmostEqual(body["com_mm"]["com_z_mm"], 24.0, delta=1e-6)
        self.assertEqual(body["com_mm"]["side_edge_from"], "ruler")


class Refusals(unittest.TestCase):
    def test_a_file_outside_the_skill_is_refused(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            outside = Path(tmp) / "copy.jsonl"
            shutil.copyfile(recording.RECORDING, outside)
            out = call("recording_summary", {"path": str(outside)})
        self.assertTrue(out["error"])
        self.assertIn("outside", out["body"]["error"])

    def test_a_non_recording_and_a_missing_file_are_refused(self) -> None:
        table = call("spin_fractions", {"path": "parameters.json"})
        self.assertTrue(table["error"])
        self.assertIn("not a .jsonl", table["body"]["error"])
        missing = call("spin_fractions", {"path": "no-such-run.jsonl"})
        self.assertTrue(missing["error"])
        self.assertIn("no such file", missing["body"]["error"])


class Plan(unittest.TestCase):
    def test_the_dark_spin_plan(self) -> None:
        out = call("recorder_plan", {"phases": ["rest", "headlift", "spins"], "note": "dark room, floor",
                                     "out_name": "run2-dark.jsonl"})
        self.assertFalse(out["error"])
        body = out["body"]
        self.assertTrue(body["runs_nothing"])
        remote = body["argv"][2]
        self.assertEqual(body["argv"][:2], ["ssh", "WIREPOD_HOST"])
        self.assertEqual(shlex.split(remote), ["env", "PHASES=rest,headlift,spins", "NOTE=dark room, floor", "python3", "-"])
        self.assertEqual(shlex.split(body["shell"]["posix_sh"]),
                         ["ssh", "WIREPOD_HOST", remote, "<", "hardware/record_barriered.py", ">", "run2-dark.jsonl"])
        self.assertEqual(body["shell"]["windows_cmd"],
                         f'ssh WIREPOD_HOST "{remote}" < hardware\\record_barriered.py > run2-dark.jsonl')
        self.assertEqual((body["stdin_file"], body["stdout_file"]), ("hardware/record_barriered.py", "run2-dark.jsonl"))
        self.assertTrue((Path(body["run_from"]) / body["stdin_file"]).is_file())
        self.assertEqual(body["recorder_sha256"], recording.sha256(mcp_service.RECORDER))
        self.assertEqual(body["watch_it_drive"], [])
        self.assertEqual(call("recorder_plan", {"phases": ["straights"]})["body"]["watch_it_drive"], ["straights"])

    def test_edge_needs_a_ruler_distance(self) -> None:
        refused = call("recorder_plan", {"phases": ["edge"]})
        self.assertTrue(refused["error"])
        self.assertIn("edge_start_mm", refused["body"]["error"])
        planned = call("recorder_plan", {"phases": ["edge"], "edge_start_mm": 40})
        self.assertFalse(planned["error"])
        self.assertIn("EDGE_START_MM=40", shlex.split(planned["body"]["argv"][2]))

    def test_a_note_that_would_break_the_shell_line_is_refused(self) -> None:
        for note in ("its table", 'a "quoted" word', "$(reboot)", "back`tick`"):
            out = call("recorder_plan", {"phases": ["rest"], "note": note})
            self.assertTrue(out["error"], note)
            self.assertIn("note:", out["body"]["error"])

    def test_bad_phase_lists_and_output_names_are_refused(self) -> None:
        planned = call("recorder_plan", {"phases": ["rest", "spins", "wheelie"]})
        self.assertFalse(planned["error"])
        self.assertEqual(planned["body"]["watch_it_drive"], ["wheelie"])
        for args in ({"phases": ["fly"]}, {"phases": []}, {"phases": ["repeat", "rest"]}, {"phases": ["rest", "rest"]},
                     {"phases": ["wheelie", "rest"]}, {"phases": ["repeat", "wheelie"]},
                     {"phases": ["rest"], "out_name": "../run2.jsonl"}):
            self.assertTrue(call("recorder_plan", args)["error"], args)

    def test_every_recorder_phase_has_its_space_written_down(self) -> None:
        self.assertEqual(set(mcp_service.PHASE_SPACE), set(mcp_service.known_phases()))


def write_constructed_run(path: Path, truth: Dict[float, float], head: float) -> None:
    """The recorder's own command rows for PHASES=spins, with state rows built to turn at known fractions."""
    commands = [row for row in test_recorder.dry_run({"PHASES": "spins"})["rows"] if row["k"] == "c"]
    spacing = params.values(params.load())["tread_spacing_mm"]
    states: List[Dict[str, Any]] = []
    for seg in recording._wheel_segments(commands):
        speed = abs(seg.left)
        rate = truth[speed] * 2.0 * speed / spacing * (1.0 if seg.right > 0 else -1.0)
        t = seg.t_cmd
        for _ in range(1000):
            if t > seg.t_stop:
                break
            states.append({"k": "s", "t": round(t, 4), "wl": seg.left, "wr": seg.right, "head": head, "status": 0,
                           "gyro": [rate * math.sin(head), 0.0, rate * math.cos(head)], "accel": [0.0, 0.0, 9806.65]})
            t += 0.06
    rows = sorted(states + commands, key=lambda row: row["t"])
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")


if __name__ == "__main__":
    unittest.main()
