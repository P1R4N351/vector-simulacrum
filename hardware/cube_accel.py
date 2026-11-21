"""Read a Vector light cube's accelerometer directly over Bluetooth LE from a host computer. Never touches the robot.

Authored-by: CPCS (cpcs-66ffc23), 2026-11-21. Needs Python 3.9+ and the bleak package, imported only when this
runs (written to bleak's documented API: BleakScanner.discover with return_adv, BleakClient read_gatt_char,
write_gatt_char, start_notify; not yet run against bleak or a cube). Writes JSON lines to stdout.

Why a host and not the robot. The robot receives the cube's raw samples but keeps them: the engine turns each one
into mm/s^2 for its own listeners (engine/components/cubes/cubeAccelComponent.cpp, HandleCubeAccelData), the SDK
passes on only events (tapped, moved, up axis), and the engine's "cubes" webviz module sends connection state,
battery and scan results, not samples (cubeCommsComponent.cpp, SendDataToWebViz). So this script takes the
robot's place and connects to the cube itself. The robot must not be holding the cube: a connected cube does not
advertise, so the scan will not find it.

What the cube sends, on the app read characteristic (robot/cube_firmware/app/accel.c, adc.c, protocol.h;
kercre123/victor, snowboy branch):
  command 2, accelerometer, 20 bytes: [2, tap_count, x0 y0 z0, x1 y1 z1, x2 y2 z2], each axis int16 little-endian.
     The app reads the accelerometer FIFO once per 10 ms tick and keeps only the last frame read in that tick,
     then sends three ticks per message.
  command 3, accelerometer failure, 2 bytes: [3, chip_id], sent when the chip id is not 0xFA.
  command 4, battery, 4 bytes: [4, unused, rail_v1 uint16 LE], every 100 ticks; volts = rail_v1 * 3.6 / 1024
     (Anki's robot/cube_firmware/bletool/index.js).
Scale: 0x2000 counts per g (bletool/index.js, ONEG). The engine divides by 32767/4 instead, 3e-5 different.
Rows keep the raw counts; nothing is converted on the way in.

How it connects. Anki's bletool (cube.js, index.js) and the robot's own client
(cubeBleClient/bleClient/bleClient.cpp) do the same thing: read the app version characteristic; if it is not
the expected 16-character version, write the app file from byte 16 on, 20 bytes per write, with response, to
the OTA characteristic, and wait for the version characteristic to report the expected version; then take
notifications from the app read characteristic. Nothing has to be written to start the samples. This script
never writes the app write characteristic, which carries light commands, so it never turns the LEDs or their
boost converter on.

Modes (CUBE_MODE):
  scan     (default) list the cubes advertising the cube service: "s" rows with address, name, RSSI and
           manufacturer data. Listens only; never connects. The cube advertises whether or not anyone listens.
  collect  connect to one cube and record for COLLECT_S seconds. It runs the cube's accelerometer and radio the
           whole time, so it refuses unless CUBE_POWER is "supply" or "rechargeable". The operator, 2026-11-21: "don't spend
           the battery until i establish either a supply or a bunch of rechargable ones".

Rows: "c" control, "s" scan result, "a" accelerometer message (tap count and 9 raw counts), "v" battery (raw rail
reading, volts, UTC), "f" accelerometer failure, "u" any other message (hex). Each row's "t" is host monotonic
seconds from the start, taken when the notification arrived, so it carries the radio's batching; the sample
clock is the cube's 10 ms tick and has to be fitted from the message count.

Settings: CUBE_MODE, CUBE_POWER, CUBE_ADDRESS (needed when more than one cube is heard; on macOS it is the
CoreBluetooth identifier from a scan, since macOS hides Bluetooth addresses), SCAN_S (2..60 s, default 10),
COLLECT_S (10..7200 s, default 120), CUBE_LOW_V (0..1.6 V, default 1.1, chosen: three readings in a row below it
end the run), CUBE_DFU (the app file, used only on a version mismatch; unit 3 keeps it at
/anki/data/assets/cozmo_resources/assets/cubeFirmware/cube.dfu), CUBE_DFU_SHA256 (default: the hash of unit 3's file),
NOTE.

Exit codes: 0 done, 3 another collector holds the lock, 4 no single cube to connect to, 5 the app check failed
(version mismatch with no file, a file with the wrong hash, or an upload the cube never confirmed), 6 the
connection failed or dropped (rows up to then are kept). SIGTERM, SIGINT and SIGHUP end a collection cleanly.
"""
import asyncio
import datetime
import hashlib
import json
import os
import signal
import struct
import sys
import time
from pathlib import Path

SERVICE = "c6f6c70f-d219-598b-fb4c-308e1f22f830"      # bletool/factory.js CUBE_SERVICE
OTA_TARGET = "9590ba9c-5140-92b5-1844-5f9d681557a4"   # bletool/cube.js OTA_TARGET_UUID
APP_VERSION = "450aa175-8d85-16a6-9148-d50e2eb7b79e"  # bletool/cube.js APP_VERSION_UUID
APP_WRITE = "0ea75290-6759-a58d-7948-598c4e02d94a"    # bletool/cube.js APP_WRITE_UUID: lights; never written here
APP_READ = "43ef14af-5fb1-7b81-3647-2a9477824cab"     # bletool/cube.js APP_READ_UUID
STOCK_VERSION = "2018-08-14T17:40"  # measured: the first 16 bytes of unit 3's cube.dfu
STOCK_SHA256 = "f22829db489a034360e78c89c27e36eab47e4007b13eb13524485467a18b633d"  # measured: unit 3's cube.dfu
APP_OFFSET, CHUNK = 16, 20          # bleClient.cpp kFirmwareVersionStrLen; bletool/cube.js MAX_BYTES_PER_PACKET
CMD_ACCEL, CMD_ACCEL_FAILURE, CMD_VOLTS = 2, 3, 4   # protocol.h
COUNTS_PER_G = 0x2000               # bletool/index.js ONEG
VOLTS_PER_COUNT = 3.6 / 1024.0      # bletool/index.js
UPLOAD_WAIT_S = 15.0                # the robot's kFirmwareUpdateTimeout_sec (cubeBleClient_vicos.cpp)
TICK_S = 0.25
LOW_READINGS = 3
LOCK = "/tmp/vector-cube-accel.lock"

MODE = os.environ.get("CUBE_MODE", "scan")
assert MODE in ("scan", "collect"), "CUBE_MODE is scan or collect"
POWER = os.environ.get("CUBE_POWER", "")
assert MODE == "scan" or POWER in ("supply", "rechargeable"), \
    "collect runs the cube: set CUBE_POWER=supply or CUBE_POWER=rechargeable once one is in place"
ADDRESS = os.environ.get("CUBE_ADDRESS", "").strip().lower()
assert len(ADDRESS) <= 64, "CUBE_ADDRESS is at most 64 characters"
SCAN_S = float(os.environ.get("SCAN_S", "10") or 0.0)
assert 2.0 <= SCAN_S <= 60.0, "SCAN_S must be 2..60 s"
COLLECT_S = float(os.environ.get("COLLECT_S", "120") or 0.0)
assert 10.0 <= COLLECT_S <= 7200.0, "COLLECT_S must be 10..7200 s"
LOW_V = float(os.environ.get("CUBE_LOW_V", "1.1") or 0.0)
assert 0.0 <= LOW_V <= 1.6, "CUBE_LOW_V must be 0..1.6 V"
DFU = os.environ.get("CUBE_DFU", "")
DFU_SHA256 = (os.environ.get("CUBE_DFU_SHA256", "") or STOCK_SHA256).lower()
assert len(DFU_SHA256) == 64 and all(c in "0123456789abcdef" for c in DFU_SHA256), "CUBE_DFU_SHA256 is 64 hex digits"

now = time.monotonic
nap = asyncio.sleep
T0 = now()
STATE = {"stop": None, "closing": False, "version": None, "low": 0}
COUNTS = {"a": 0, "v": 0, "f": 0, "u": 0}
SPAN = {"first": None, "last": None}


def emit(kind, **fields):
    row = {"k": kind, "t": round(now() - T0, 4)}
    row.update(fields)
    sys.stdout.write(json.dumps(row) + "\n")
    sys.stdout.flush()


def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds")


def decode(data):
    """One message from the cube as (row kind, fields), by the layouts in protocol.h."""
    data = bytes(data)
    if len(data) == 20 and data[0] == CMD_ACCEL:
        return "a", {"tap": data[1], "raw": list(struct.unpack_from("<9h", data, 2))}
    if len(data) == 4 and data[0] == CMD_VOLTS:
        rail = struct.unpack_from("<H", data, 2)[0]
        return "v", {"rail": rail, "volts": round(rail * VOLTS_PER_COUNT, 4)}
    if len(data) == 2 and data[0] == CMD_ACCEL_FAILURE:
        return "f", {"chip_id": data[1]}
    return "u", {"n": len(data), "hex": data.hex()[:80]}


def to_g(raw):
    """Raw counts to g."""
    return [float(r) / COUNTS_PER_G for r in raw]


def app_chunks(app):
    """The writes that load an app file: from byte 16 to the end, 20 bytes at a time."""
    assert len(app) > APP_OFFSET, "an app file is longer than its 16-byte version"
    return [app[i:i + CHUNK] for i in range(APP_OFFSET, len(app), CHUNK)]


def on_app_data(_sender, data):
    kind, fields = decode(data)
    COUNTS[kind] += 1
    if kind == "a":
        SPAN["first"] = now() if SPAN["first"] is None else SPAN["first"]
        SPAN["last"] = now()
    if kind == "v":
        fields["utc"] = utc()
        STATE["low"] = STATE["low"] + 1 if fields["volts"] < LOW_V else 0
        if STATE["low"] >= LOW_READINGS:
            STATE["stop"] = "low_cube_battery"
    emit(kind, **fields)


def on_version(_sender, data):
    STATE["version"] = bytes(data).decode("utf-8", "replace")


def on_disconnect(_client):
    if not STATE["closing"]:
        STATE["stop"] = "disconnected"


def on_signal(signum, _frame):
    STATE["stop"] = signal.Signals(signum).name


def load_app():
    """(ok, app bytes or None). No file is fine until an upload is needed; a file with the wrong hash is not."""
    if not DFU:
        return True, None
    app = Path(DFU).read_bytes()
    digest = hashlib.sha256(app).hexdigest()
    if digest != DFU_SHA256:
        emit("c", name="app_file_refused", path=DFU[-120:], sha256=digest, expected=DFU_SHA256)
        return False, None
    return True, app


async def read_version(client):
    STATE["version"] = bytes(await client.read_gatt_char(APP_VERSION)).decode("utf-8", "replace")
    return STATE["version"]


async def check_app(client, expected, app):
    """True when the cube runs the expected app, uploading it first if the version differs and a file is given."""
    await client.start_notify(APP_VERSION, on_version)
    version = await read_version(client)
    emit("c", name="app_version", version=version[:40], expected=expected)
    if version == expected:
        return True
    if app is None:
        emit("c", name="app_mismatch_no_file")
        return False
    chunks = app_chunks(app)
    for chunk in chunks:
        await client.write_gatt_char(OTA_TARGET, chunk, response=True)
    emit("c", name="app_uploaded", writes=len(chunks), bytes=len(app) - APP_OFFSET)
    for _ in range(int(UPLOAD_WAIT_S / TICK_S) + 1):
        if STATE["version"] == expected:
            break
        await nap(TICK_S)
    if STATE["version"] != expected:
        await read_version(client)
    confirmed = STATE["version"] == expected
    emit("c", name="app_confirmed" if confirmed else "app_not_confirmed", version=str(STATE["version"])[:40])
    return confirmed


async def collect(client):
    """Take the cube's messages for COLLECT_S seconds, or until a stop."""
    await client.start_notify(APP_READ, on_app_data)
    start = now()
    emit("c", name="collecting", seconds=COLLECT_S, utc=utc())
    for _ in range(int(COLLECT_S / TICK_S) + 1):
        if STATE["stop"] or now() - start >= COLLECT_S:
            break
        await nap(TICK_S)
    if STATE["stop"] != "disconnected":
        try:
            await client.stop_notify(APP_READ)
        except Exception as exc:  # recorded; the disconnect that follows ends the notifications anyway
            emit("c", name="stop_notify_failed", error=type(exc).__name__)
    emit("c", name="collected", seconds=round(now() - start, 3), stopped_by=STATE["stop"], utc=utc())


async def scan(ble):
    """Every device advertising the cube service, as "s" rows; the devices themselves are returned."""
    found = await ble.BleakScanner.discover(timeout=SCAN_S, return_adv=True, service_uuids=[SERVICE])
    cubes = []
    for address in sorted(found):
        device, adv = found[address]
        if SERVICE not in [str(u).lower() for u in (adv.service_uuids or [])]:
            continue
        mfr = {str(k): bytes(v).hex()[:40] for k, v in (adv.manufacturer_data or {}).items()}
        emit("s", address=str(address)[:64], name=str(adv.local_name or "")[:40], rssi=int(adv.rssi), mfr=mfr)
        cubes.append(device)
    return cubes


def pick(cubes):
    """The cube to connect to: the one named by CUBE_ADDRESS, or the only one heard."""
    if ADDRESS:
        named = [d for d in cubes if str(d.address).lower() == ADDRESS]
        return named[0] if len(named) == 1 else None
    return cubes[0] if len(cubes) == 1 else None


def summary():
    n, first, last = COUNTS["a"], SPAN["first"], SPAN["last"]
    period = round((last - first) / (n - 1), 5) if n > 1 else None
    return {"accel_messages": n, "battery_rows": COUNTS["v"], "failures": COUNTS["f"], "other": COUNTS["u"],
            "mean_message_period_s": period}


async def session(ble, device, app):
    """Connect, check the app, collect, disconnect. Returns the exit code."""
    client = ble.BleakClient(device, disconnected_callback=on_disconnect)
    try:
        await client.connect()
    except Exception as exc:  # reported as the exit code; nothing was connected, so nothing to undo
        emit("c", name="connect_failed", error=("%s: %s" % (type(exc).__name__, exc))[:160])
        return 6
    emit("c", name="connected", address=str(device.address)[:64], utc=utc())
    try:
        expected = app[:APP_OFFSET].decode("utf-8", "replace") if app is not None else STOCK_VERSION
        if not await check_app(client, expected, app):
            return 5
        await collect(client)
    finally:
        STATE["closing"] = True
        try:
            await client.disconnect()
            emit("c", name="disconnected")
        except Exception as exc:  # the cube drops the link itself when the host goes away
            emit("c", name="disconnect_failed", error=type(exc).__name__)
        emit("c", name="end", utc=utc(), **summary())
    return 6 if STATE["stop"] == "disconnected" else 0


async def run(ble):
    ok, app = load_app()
    emit("c", name="begin", mode=MODE, power=POWER or None, platform=sys.platform, counts_per_g=COUNTS_PER_G,
         volts_per_count=VOLTS_PER_COUNT, app_file=bool(app), bleak=str(getattr(ble, "__version__", "unknown")),
         note=os.environ.get("NOTE", "")[:400], utc=utc())
    if not ok:
        return 5
    cubes = await scan(ble)
    emit("c", name="scanned", cubes=len(cubes), seconds=SCAN_S)
    if MODE == "scan":
        return 0
    device = pick(cubes)
    if device is None:
        emit("c", name="no_single_cube", heard=len(cubes), address=ADDRESS or None)
        return 4
    return await session(ble, device, app)


def main():
    try:
        fd = os.open(LOCK, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    except FileExistsError:
        emit("c", name="lock_held", path=LOCK)
        return 3
    os.write(fd, ("cpcs-66ffc23 cube accelerometer pid %d\n" % os.getpid()).encode())
    os.close(fd)
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, on_signal)
    try:
        import bleak
        return asyncio.run(run(bleak))
    finally:
        os.remove(LOCK)


if __name__ == "__main__":
    sys.exit(main())
