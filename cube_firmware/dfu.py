"""Read, check, patch and write the light cube's app file, cube.dfu..

    python3 cube_firmware/dfu.py --stock cube.dfu --key key.h --version build-avg1-261121 --out cube-avg.dfu

The layout (robot/cube_firmware/tools/sign.py, checked here against unit 3's file): the 16-byte version, the app
image XXTEA-encrypted as one block under the 16-byte key in robot/cube_firmware/tools/key.h, then one 0xFF. The
robot sends everything after the first 16 bytes, 20 per write. The bootloader copies each write's length rounded
down to a multiple of 4, and a write whose length is not a multiple of 4 ends the upload: that is what the
trailing 0xFF is for. It then zeroes the rest of the app region, decrypts what it received in place, and starts
the app only if the 16 bytes at 0x24 are its nonce. It reports the decrypted image's first 16 bytes as the
version, which the robot compares with the file's first 16 (user_custs1_impl.c,
user_custs1_ota_target_wr_ind_handler; bleClient.cpp OnReceiveMessage), so the two must be equal.

The patch: the image as shipped, with its version replaced and the accelerometer tick at 0x20005db4 replaced by
tick_avg.s, assembled and linked by the ARM GNU toolchain. Nothing else changes, and the script checks that.
The key is read from the file given and never printed. Exit codes: 0 written, 2 bad arguments, 3 a check failed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import List, Sequence, Tuple

BASE = 0x20005800                 # application.sct: the app region
REGION = 0x27C0                   # application.sct: its size
NONCE = bytes.fromhex("3d707a86103cef4f832fa8a66420badf")  # user_custs1_impl.c xxtea_nonce; app/boot.s
NONCE_AT = 0x24                   # application_api.h: after the version and five pointers
VERSION_LEN = 16
STOCK_SHA256 = "f22829db489a034360e78c89c27e36eab47e4007b13eb13524485467a18b633d"  # unit 3's cube.dfu
STOCK_VERSION = b"2018-08-14T17:40"
TICK_AT, TICK_END = 0x20005db4, 0x20005e80  # the shipped hal_acc_tick and its literal pool
DELTA = 0x9E3779B9
MASK = 0xFFFFFFFF
HERE = Path(__file__).resolve().parent
TOOLS = ("arm-none-eabi-as", "arm-none-eabi-ld", "arm-none-eabi-objcopy")


class CheckFailed(Exception):
    """The file, the key or the patch is not what it must be."""


def read_key(path: Path) -> List[int]:
    """key.h's 16 comma-separated bytes as the four little-endian words the bootloader uses."""
    values = [int(c, 16) for c in re.split(r"\s*,\s*", path.read_text().strip().rstrip(","))]
    if len(values) != 16 or not all(0 <= v <= 255 for v in values):
        raise CheckFailed("the key file must hold 16 bytes")
    return list(int.from_bytes(bytes(values[i:i + 4]), "little") for i in range(0, 16, 4))


def _mx(z: int, y: int, total: int, p: int, e: int, key: Sequence[int]) -> int:
    # user_custs1_impl.c: (((z>>5^y<<2) + (y>>3^z<<4)) ^ ((sum^y) + (key[(p&3)^e] ^ z)))
    left = (((z >> 5) ^ ((y << 2) & MASK)) + ((y >> 3) ^ ((z << 4) & MASK))) & MASK
    right = ((total ^ y) + (key[(p & 3) ^ e] ^ z)) & MASK
    return left ^ right


def _words(data: bytes) -> List[int]:
    if len(data) % 4 or len(data) < 8:
        raise CheckFailed("XXTEA needs at least two whole words")
    return [int.from_bytes(data[i:i + 4], "little") for i in range(0, len(data), 4)]


def _bytes(words: Sequence[int]) -> bytes:
    return b"".join(w.to_bytes(4, "little") for w in words)


def decrypt(data: bytes, key: Sequence[int]) -> bytes:
    """The bootloader's xxtea(), word for word."""
    v = _words(data)
    n = len(v)
    rounds = 6 + 52 // n
    total = (rounds * DELTA) & MASK
    y = v[0]
    for _ in range(rounds):
        e = (total >> 2) & 3
        for p in range(n - 1, 0, -1):
            z = v[p - 1]
            v[p] = (v[p] - _mx(z, y, total, p, e, key)) & MASK
            y = v[p]
        z = v[n - 1]
        v[0] = (v[0] - _mx(z, y, total, 0, e, key)) & MASK
        y = v[0]
        total = (total - DELTA) & MASK
    return _bytes(v)


def encrypt(data: bytes, key: Sequence[int]) -> bytes:
    """The inverse of decrypt: XXTEA's encoding direction."""
    v = _words(data)
    n = len(v)
    total, z = 0, v[n - 1]
    for _ in range(6 + 52 // n):
        total = (total + DELTA) & MASK
        e = (total >> 2) & 3
        for p in range(n - 1):
            y = v[p + 1]
            v[p] = (v[p] + _mx(z, y, total, p, e, key)) & MASK
            z = v[p]
        y = v[0]
        v[n - 1] = (v[n - 1] + _mx(z, y, total, n - 1, e, key)) & MASK
        z = v[n - 1]
    return _bytes(v)


def open_dfu(raw: bytes, key: Sequence[int]) -> bytes:
    """The decrypted image of a cube.dfu, after every check the bootloader and the robot make."""
    if len(raw) < VERSION_LEN + 9 or raw[-1] != 0xFF or (len(raw) - VERSION_LEN - 1) % 4:
        raise CheckFailed("not a cube.dfu: version, whole words of image, then 0xFF")
    image = decrypt(raw[VERSION_LEN:-1], key)
    if image[NONCE_AT:NONCE_AT + 16] != NONCE:
        raise CheckFailed("the nonce does not decrypt: wrong key or a damaged file")
    if image[:VERSION_LEN] != raw[:VERSION_LEN]:
        raise CheckFailed("the image's version differs from the file's, so the robot would report a failed flash")
    if len(image) > REGION:
        raise CheckFailed("the image is larger than the app region")
    return image


def make_dfu(image: bytes, key: Sequence[int]) -> bytes:
    raw = image[:VERSION_LEN] + encrypt(image, key) + b"\xff"
    if open_dfu(raw, key) != image:
        raise CheckFailed("the written file does not decrypt back to the image")
    return raw


def assemble(source: Path, script: Path) -> bytes:
    """tick_avg.s assembled and linked at TICK_AT, as raw bytes; it must fit in TICK_END - TICK_AT."""
    missing = [t for t in TOOLS if shutil.which(t) is None]
    if missing:
        raise CheckFailed("the ARM GNU toolchain is missing: " + ", ".join(missing))
    with tempfile.TemporaryDirectory() as tmp:
        obj, elf, out = Path(tmp) / "tick.o", Path(tmp) / "tick.elf", Path(tmp) / "tick.bin"
        steps = ([TOOLS[0], "-mcpu=cortex-m0", "-mthumb", "-mno-warn-deprecated", "--fatal-warnings", str(source),
                  "-o", str(obj)],
                 [TOOLS[1], "-T", str(script), str(obj), "-o", str(elf), "--no-warn-rwx-segments"],
                 [TOOLS[2], "-O", "binary", "-j", ".text", str(elf), str(out)])
        for step in steps:
            done = subprocess.run(step, capture_output=True, text=True, timeout=60)
            if done.returncode != 0:
                raise CheckFailed("%s failed: %s" % (Path(step[0]).name, (done.stderr or done.stdout).strip()[:400]))
        code = out.read_bytes()
    if not 0 < len(code) <= TICK_END - TICK_AT:
        raise CheckFailed("the tick is %d bytes; the shipped tick's place holds %d" % (len(code), TICK_END - TICK_AT))
    return code


def patch(image: bytes, version: bytes, tick: bytes) -> bytes:
    """The image with a new version and tick; the rest of the old tick's place is zeroed."""
    if len(version) != VERSION_LEN:
        raise CheckFailed("the version is exactly 16 bytes")
    at, end = TICK_AT - BASE, TICK_END - BASE
    out = bytearray(image)
    out[:VERSION_LEN] = version
    out[at:end] = tick + bytes(end - at - len(tick))
    return bytes(out)


def changed_ranges(a: bytes, b: bytes) -> List[Tuple[int, int]]:
    """Address ranges [start, end) where two images of equal length differ."""
    assert len(a) == len(b), "images of equal length"
    out: List[Tuple[int, int]] = []
    for i in range(len(a)):
        if a[i] != b[i]:
            if out and out[-1][1] == BASE + i:
                out[-1] = (out[-1][0], BASE + i + 1)
            else:
                out.append((BASE + i, BASE + i + 1))
    return out


def build(stock_raw: bytes, key: Sequence[int], version: bytes) -> Tuple[bytes, bytes, bytes]:
    """(stock image, patched image, patched file), after every check."""
    if hashlib.sha256(stock_raw).hexdigest() != STOCK_SHA256:
        raise CheckFailed("the stock file is not unit 3's cube.dfu (its SHA-256 differs)")
    stock = open_dfu(stock_raw, key)
    if stock[:VERSION_LEN] != STOCK_VERSION:
        raise CheckFailed("the stock image's version is not 2018-08-14T17:40")
    if stock_raw[:VERSION_LEN] == version:
        raise CheckFailed("the new version must differ from the stock one, or the robot never flashes it")
    image = patch(stock, version, assemble(HERE / "tick_avg.s", HERE / "link.ld"))
    for start, end in changed_ranges(stock, image):
        if not (end <= BASE + VERSION_LEN or (TICK_AT <= start and end <= TICK_END)):
            raise CheckFailed("the patch changed bytes outside the version and the tick: %#x..%#x" % (start, end))
    return stock, image, make_dfu(image, key)


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="Build the cube app with the averaging tick.")
    parser.add_argument("--stock", required=True, type=Path, help="Unit 3's cube.dfu")
    parser.add_argument("--key", required=True, type=Path, help="robot/cube_firmware/tools/key.h")
    parser.add_argument("--version", required=True, help="16 characters")
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args(argv)
    version = args.version.encode("ascii", "strict")
    if len(version) != VERSION_LEN:
        print("the version must be exactly 16 ASCII characters", file=sys.stderr)
        return 2
    try:
        stock_raw = args.stock.read_bytes()
        stock, image, raw = build(stock_raw, read_key(args.key), version)
    except (CheckFailed, OSError) as exc:
        print("check failed: %s" % exc, file=sys.stderr)
        return 3
    args.out.write_bytes(raw)
    print(json.dumps({"stock_sha256": hashlib.sha256(stock_raw).hexdigest(), "out": str(args.out),
                      "out_sha256": hashlib.sha256(raw).hexdigest(), "out_bytes": len(raw),
                      "version": version.decode(), "image_bytes": len(image),
                      "changed": ["%#x-%#x" % r for r in changed_ranges(stock, image)],
                      "writes": -(-(len(raw) - VERSION_LEN) // 20)}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
