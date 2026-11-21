"""The cube accelerometer collector, run against a stand-in bleak, stand-in cubes and a stand-in clock.
script: the message layout from robot/cube_firmware/app/protocol.h (packed here with struct), the field offsets
and scale from bletool/index.js, the UUIDs as bletool/cube.js and factory.js write them, and the service UUID
from the advertising bytes in src/config/user_config.h. It says nothing about a real cube or bleak.
"""

from __future__ import annotations

import contextlib
import hashlib
import importlib.util
import io
import json
import os
import struct
import sys
import tempfile
import types
import unittest
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple
from unittest import mock

import _common  # noqa: F401  (puts the skill on the path)

SCRIPT = Path(__file__).resolve().parent.parent / "hardware" / "cube_accel.py"
ENV_KEYS = ("CUBE_MODE", "CUBE_POWER", "CUBE_ADDRESS", "SCAN_S", "COLLECT_S", "CUBE_LOW_V", "CUBE_DFU",
            "CUBE_DFU_SHA256", "NOTE")


def dashed(hex32: str) -> str:
    return "-".join((hex32[:8], hex32[8:12], hex32[12:16], hex32[16:20], hex32[20:]))


# As bletool/cube.js and factory.js write them.
NOBLE = {"SERVICE": "c6f6c70fd219598bfb4c308e1f22f830", "OTA_TARGET": "9590ba9c514092b518445f9d681557a4",
         "APP_VERSION": "450aa1758d8516a69148d50e2eb7b79e", "APP_WRITE": "0ea752906759a58d7948598c4e02d94a",
         "APP_READ": "43ef14af5fb17b8136472a9477824cab"}
SERVICE, OTA, VERSION_UUID, WRITE_UUID, READ_UUID = (dashed(NOBLE[k]) for k in
                                                     ("SERVICE", "OTA_TARGET", "APP_VERSION", "APP_WRITE", "APP_READ"))
STOCK = "2018-08-14T17:40"
ACCEL_EVERY, BATTERY_EVERY = 0.03, 1.0


def accel_packet(k: int) -> bytes:
    raw = [k, -k, 8192, 2 * k, -2 * k, 8191, 3 * k, -3 * k, 8190]
    return struct.pack("<BB9h", 2, (k // 50) % 256, *[max(-32768, min(32767, v)) for v in raw])


class World:
    """Stand-in cubes and clock. Events are (time, order, action), run in time order as the clock passes them."""

    def __init__(self, cubes: int = 1, version: str = STOCK, boots_to: Optional[str] = None,
                 boot_bytes: int = 0, rail: int = 445, drop_at: Optional[float] = None,
                 connect_error: bool = False) -> None:
        self.t, self.order, self.events = 0.0, 0, []  # type: float, int, List[Tuple[float, int, Callable[[], None]]]
        self.version, self.boots_to, self.boot_bytes, self.rail = version, boots_to, boot_bytes, rail
        self.drop_at, self.connect_error = drop_at, connect_error
        self.writes: List[Tuple[str, bytes, bool]] = []
        self.notify: Dict[str, Callable[..., None]] = {}
        self.clients: List[Any] = []
        self.scans, self.disconnects, self.streaming, self.k = 0, 0, False, 0
        self.devices = [self._device("cube-%d" % i, [SERVICE.upper()]) for i in range(cubes)]
        self.devices.append(self._device("not-a-cube", ["0000180a-0000-1000-8000-00805f9b34fb"]))

    @staticmethod
    def _device(address: str, uuids: List[str]) -> Any:
        adv = types.SimpleNamespace(service_uuids=uuids, local_name=None, manufacturer_data={0xEFBE: b"\x07\x00"},
                                    rssi=-61)
        return types.SimpleNamespace(address=address, adv=adv)

    def at(self, when: float, action: Callable[[], None]) -> None:
        self.order += 1
        self.events.append((when, self.order, action))

    def now(self) -> float:
        return self.t

    async def nap(self, seconds: float) -> None:
        target = self.t + seconds
        for _ in range(100000):
            due = [e for e in self.events if e[0] <= target + 1e-9]
            if not due:
                break
            event = min(due)
            self.events.remove(event)
            self.t = max(self.t, event[0])
            event[2]()
        self.t = target

    def start_stream(self, client: Any) -> None:
        self.streaming, start = True, self.t
        self.at(start + ACCEL_EVERY, lambda: self._accel(client, start))
        self.at(start + BATTERY_EVERY, lambda: self._battery(client, start, 1))
        if self.drop_at is not None:
            self.at(self.drop_at, lambda: self._drop(client))

    def _accel(self, client: Any, start: float) -> None:
        if not (self.streaming and client.connected):
            return
        self.k += 1
        self.notify[READ_UUID](READ_UUID, bytearray(accel_packet(self.k)))
        self.at(start + ACCEL_EVERY * (self.k + 1), lambda: self._accel(client, start))

    def _battery(self, client: Any, start: float, n: int) -> None:
        if not (self.streaming and client.connected):
            return
        self.notify[READ_UUID](READ_UUID, bytearray(struct.pack("<BBH", 4, 0, self.rail)))
        self.at(start + BATTERY_EVERY * (n + 1), lambda: self._battery(client, start, n + 1))

    def _drop(self, client: Any) -> None:
        client.connected, self.streaming = False, False
        client.cb(client)

    def ota_bytes(self) -> int:
        return sum(len(data) for uuid, data, _ in self.writes if uuid == OTA)

    def boot(self) -> None:
        self.version = str(self.boots_to)
        if VERSION_UUID in self.notify:
            self.notify[VERSION_UUID](VERSION_UUID, bytearray(self.version.encode()))


def fake_bleak(world: World) -> types.ModuleType:
    class Scanner:
        @staticmethod
        async def discover(timeout: float = 0.0, return_adv: bool = False, service_uuids: Any = None) -> Any:
            assert return_adv, "the collector needs the advertisement data"
            world.scans += 1
            return {d.address: (d, d.adv) for d in world.devices}

    class Client:
        def __init__(self, device: Any, disconnected_callback: Any = None) -> None:
            self.device, self.cb, self.connected = device, disconnected_callback, False
            world.clients.append(self)

        async def connect(self) -> None:
            if world.connect_error:
                raise TimeoutError("no answer")
            self.connected = True

        async def disconnect(self) -> None:
            self.connected, world.streaming = False, False
            world.disconnects += 1

        async def read_gatt_char(self, uuid: str) -> bytearray:
            assert uuid == VERSION_UUID, "only the version is ever read"
            return bytearray(world.version.encode())

        async def write_gatt_char(self, uuid: str, data: bytes, response: bool = False) -> None:
            world.writes.append((uuid, bytes(data), response))
            if world.boots_to is not None and uuid == OTA and world.ota_bytes() == world.boot_bytes:
                world.at(world.t + 0.1, world.boot)

        async def start_notify(self, uuid: str, callback: Callable[..., None]) -> None:
            world.notify[uuid] = callback
            if uuid == READ_UUID:
                world.start_stream(self)

        async def stop_notify(self, uuid: str) -> None:
            world.notify.pop(uuid, None)
            if uuid == READ_UUID:
                world.streaming = False

    module = types.ModuleType("bleak")
    module.BleakScanner, module.BleakClient, module.__version__ = Scanner, Client, "stand-in"
    return module


def load(env: Dict[str, str]) -> Any:
    """The script, loaded under env, as a module; a settings assertion propagates."""
    with mock.patch.dict(os.environ, env):
        for key in ENV_KEYS:
            if key not in env:
                os.environ.pop(key, None)
        spec = importlib.util.spec_from_file_location("cube_accel_under_test", SCRIPT)
        assert spec is not None and spec.loader is not None, "cannot load the script"
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


def run(env: Dict[str, str], world: World, lock_held: bool = False) -> Dict[str, Any]:
    module = load(env)
    out = io.StringIO()
    with tempfile.TemporaryDirectory() as tmp, mock.patch.dict(sys.modules, {"bleak": fake_bleak(world)}):
        module.now, module.nap, module.T0, module.LOCK = world.now, world.nap, 0.0, str(Path(tmp) / "lock")
        if lock_held:
            Path(module.LOCK).write_text("held")
        with contextlib.redirect_stdout(out):
            code = module.main()
        lock_left = Path(module.LOCK).exists() and not lock_held
    rows = [json.loads(line) for line in out.getvalue().splitlines()]
    return {"rows": rows, "code": code, "lock_left": lock_left, "module": module}


def kinds(rows: List[Dict[str, Any]], kind: str) -> List[Dict[str, Any]]:
    return [row for row in rows if row["k"] == kind]


def control(rows: List[Dict[str, Any]], name: str) -> List[Dict[str, Any]]:
    return [row for row in rows if row["k"] == "c" and row.get("name") == name]


def app_file(tmp: str, version: str = "2026-11-21T01:23") -> Tuple[str, bytes]:
    body = bytes((i * 37 + 11) % 256 for i in range(3209 - 16))
    data = version.encode() + body
    path = Path(tmp) / "cube.dfu"
    path.write_bytes(data)
    return str(path), data


COLLECT = {"CUBE_MODE": "collect", "CUBE_POWER": "supply", "COLLECT_S": "10"}


class Protocol(unittest.TestCase):
    def test_an_accelerometer_message_decodes_as_protocol_h_lays_it_out(self) -> None:
        module = load({})
        packet = struct.pack("<BB9h", 2, 7, 8192, -4096, 123, -1, 0, 32767, -32768, 16, 4095)
        kind, fields = module.decode(bytearray(packet))
        self.assertEqual(kind, "a")
        self.assertEqual(fields["tap"], 7)
        self.assertEqual(fields["raw"], [8192, -4096, 123, -1, 0, 32767, -32768, 16, 4095])
        # bletool/index.js reads the first sample with readInt16LE at offsets 2, 4 and 6, divided by 0x2000
        first = [int.from_bytes(packet[o:o + 2], "little", signed=True) / 0x2000 for o in (2, 4, 6)]
        self.assertEqual(module.to_g(fields["raw"][:3]), first)
        self.assertEqual(first, [1.0, -0.5, 123 / 8192])

    def test_battery_failure_and_other_messages(self) -> None:
        module = load({})
        self.assertEqual(module.decode(struct.pack("<BBH", 4, 0, 445)), ("v", {"rail": 445, "volts": 1.5645}))
        self.assertEqual(module.decode(bytes([3, 0x12])), ("f", {"chip_id": 0x12}))
        self.assertEqual(module.decode(bytes([2, 0, 1, 2])), ("u", {"n": 4, "hex": "02000102"}))
        self.assertEqual(module.decode(bytes([9])), ("u", {"n": 1, "hex": "09"}))

    def test_the_uuids_are_anki_s_and_the_service_is_the_advertised_one(self) -> None:
        module = load({})
        got = (module.SERVICE, module.OTA_TARGET, module.APP_VERSION, module.APP_WRITE, module.APP_READ)
        self.assertEqual(got, (SERVICE, OTA, VERSION_UUID, WRITE_UUID, READ_UUID))
        advertised = b"\x30\xF8\x22\x1F\x8E\x30\x4C\xFB\x8B\x59\x19\xD2\x0F\xC7\xF6\xC6"  # user_config.h, LSB first
        self.assertEqual(dashed(advertised[::-1].hex()), module.SERVICE)

    def test_an_app_file_is_written_from_byte_16_in_writes_of_at_most_20_bytes(self) -> None:
        module = load({})
        with tempfile.TemporaryDirectory() as tmp:
            _, data = app_file(tmp)
        chunks = module.app_chunks(data)
        self.assertEqual(len(chunks), 160)  # (3209 - 16) / 20, rounded up
        self.assertTrue(all(1 <= len(c) <= 20 for c in chunks))
        self.assertEqual(b"".join(chunks), data[16:])


class Collector(unittest.TestCase):
    def test_scan_lists_the_cubes_and_never_connects(self) -> None:
        world = World(cubes=2)
        got = run({}, world)
        self.assertEqual(got["code"], 0)
        self.assertEqual([r["address"] for r in kinds(got["rows"], "s")], ["cube-0", "cube-1"])
        self.assertEqual(kinds(got["rows"], "s")[0]["mfr"], {str(0xEFBE): "0700"})
        self.assertEqual(world.clients, [])
        self.assertFalse(got["lock_left"])

    def test_collect_requires_an_explicit_power_source_before_radio_use(self) -> None:
        for power in ("", "battery", "alkaline"):
            env = {"CUBE_MODE": "collect", "CUBE_POWER": power} if power else {"CUBE_MODE": "collect"}
            with self.assertRaises(AssertionError, msg=power):
                load(env)

    def test_a_matching_version_collects_messages_and_writes_nothing(self) -> None:
        world = World()
        got = run(COLLECT, world)
        self.assertEqual(got["code"], 0)
        self.assertEqual(world.writes, [], "nothing is ever written when the app already matches")
        accel = kinds(got["rows"], "a")
        self.assertEqual(len(accel), int(10.0 / ACCEL_EVERY + 1e-9))
        self.assertEqual(accel[0]["raw"], [1, -1, 8192, 2, -2, 8191, 3, -3, 8190])
        self.assertEqual(len(kinds(got["rows"], "v")), int(10.0 / BATTERY_EVERY + 1e-9))
        end = control(got["rows"], "end")[0]
        self.assertEqual(end["accel_messages"], len(accel))
        self.assertAlmostEqual(end["mean_message_period_s"], ACCEL_EVERY, places=4)
        self.assertEqual(control(got["rows"], "collected")[0]["stopped_by"], None)
        self.assertEqual(world.disconnects, 1)
        self.assertFalse(got["lock_left"])

    def test_disposable_run_can_read_the_known_averaging_app_without_writing(self) -> None:
        world = World(version="build-avg1-261121")
        got = run(dict(COLLECT, CUBE_POWER="disposable"), world)
        self.assertEqual(got["code"], 0)
        self.assertTrue(kinds(got["rows"], "a"))
        self.assertEqual(world.writes, [])
        self.assertEqual(control(got["rows"], "begin")[0]["power"], "disposable")

    def test_a_version_mismatch_uploads_the_app_from_byte_16_with_response_then_collects(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path, data = app_file(tmp)
            world = World(version="2017-01-01T00:00", boots_to=data[:16].decode(), boot_bytes=len(data) - 16)
            env = dict(COLLECT, CUBE_DFU=path, CUBE_DFU_SHA256=hashlib.sha256(data).hexdigest())
            got = run(env, world)
        self.assertEqual(got["code"], 0)
        self.assertEqual({uuid for uuid, _, _ in world.writes}, {OTA}, "only the OTA characteristic is written")
        self.assertTrue(all(response for _, _, response in world.writes), "every app write waits for its response")
        self.assertEqual(b"".join(d for _, d, _ in world.writes), data[16:])
        self.assertEqual(len(control(got["rows"], "app_confirmed")), 1)
        self.assertGreater(len(kinds(got["rows"], "a")), 300)

    def test_an_upload_the_cube_never_confirms_stops_before_collecting(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path, data = app_file(tmp)
            world = World(version="2017-01-01T00:00")
            got = run(dict(COLLECT, CUBE_DFU=path, CUBE_DFU_SHA256=hashlib.sha256(data).hexdigest()), world)
        self.assertEqual(got["code"], 5)
        self.assertNotIn(READ_UUID, world.notify)
        self.assertEqual(kinds(got["rows"], "a"), [])
        self.assertEqual(len(control(got["rows"], "app_not_confirmed")), 1)
        self.assertEqual(world.disconnects, 1)

    def test_a_mismatch_with_no_app_file_stops_without_writing(self) -> None:
        world = World(version="2017-01-01T00:00")
        got = run(COLLECT, world)
        self.assertEqual(got["code"], 5)
        self.assertEqual(world.writes, [])
        self.assertEqual(world.disconnects, 1)

    def test_an_app_file_with_another_hash_is_refused_before_the_radio_is_used(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path, _ = app_file(tmp)
            world = World()
            got = run(dict(COLLECT, CUBE_DFU=path), world)  # default hash: unit 3's file, which this is not
        self.assertEqual(got["code"], 5)
        self.assertEqual((world.scans, world.clients), (0, []))
        self.assertEqual(len(control(got["rows"], "app_file_refused")), 1)

    def test_several_cubes_and_no_address_means_no_connection_but_an_address_picks_one(self) -> None:
        world = World(cubes=2)
        got = run(COLLECT, world)
        self.assertEqual(got["code"], 4)
        self.assertEqual(world.clients, [])
        world = World(cubes=2)
        got = run(dict(COLLECT, CUBE_ADDRESS="CUBE-1"), world)
        self.assertEqual(got["code"], 0)
        self.assertEqual([c.device.address for c in world.clients], ["cube-1"])

    def test_battery_without_accelerometer_times_out(self) -> None:
        world = World()
        with mock.patch.object(world, "_accel"):
            got = run(COLLECT, world)
        self.assertEqual(got["code"], 6)
        self.assertTrue(kinds(got["rows"], "v"))
        self.assertEqual(control(got["rows"], "collected")[0]["stopped_by"], "accel_stream_timeout")
        self.assertEqual(world.disconnects, 1)

    def test_a_dropped_connection_ends_the_run_and_keeps_what_arrived(self) -> None:
        world = World(drop_at=5.0)
        got = run(COLLECT, world)
        self.assertEqual(got["code"], 6)
        self.assertEqual(control(got["rows"], "collected")[0]["stopped_by"], "disconnected")
        self.assertEqual(len(kinds(got["rows"], "a")), int(5.0 / ACCEL_EVERY + 1e-9))

    def test_a_failed_connection_is_reported_and_nothing_else_happens(self) -> None:
        world = World(connect_error=True)
        got = run(COLLECT, world)
        self.assertEqual(got["code"], 6)
        self.assertEqual((world.notify, world.writes), ({}, []))

    def test_three_low_battery_readings_end_the_run(self) -> None:
        world = World(rail=250)  # 0.879 V by bletool's scale
        got = run(COLLECT, world)
        self.assertEqual(got["code"], 0)
        collected = control(got["rows"], "collected")[0]
        self.assertEqual(collected["stopped_by"], "low_cube_battery")
        self.assertLess(collected["seconds"], 3.5)
        self.assertEqual(len(kinds(got["rows"], "v")), 3)

    def test_a_held_lock_means_no_radio_use(self) -> None:
        world = World()
        got = run(COLLECT, world, lock_held=True)
        self.assertEqual(got["code"], 3)
        self.assertEqual((world.scans, world.clients), (0, []))

    def test_settings_out_of_range_are_refused(self) -> None:
        for env in ({"SCAN_S": "1"}, {"SCAN_S": "61"}, {"COLLECT_S": "5"}, {"COLLECT_S": "8000"},
                    {"CUBE_LOW_V": "2"}, {"CUBE_MODE": "stream"}, {"CUBE_DFU_SHA256": "abc"}):
            with self.assertRaises(AssertionError, msg=str(env)):
                load(env)


if __name__ == "__main__":
    unittest.main()
