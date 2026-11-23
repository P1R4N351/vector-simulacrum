#!/usr/bin/env python3
"""Fetch and verify unmodified WireOS Light Cube and STM32 syscon firmware into public/robot/firmware/.

Downloads pinned upstream images from os-vector/wire-os-victor at commit
3cec4844d24c4719b5b3892e4979494bcd260424 (never committing binary blobs or
proprietary decryption keys into this repository):
  - resources/assets/cubeFirmware/cube.dfu -> decrypted in memory to cube.plain.bin
  - robot/fixture/stm/releases/sysboot.bin + syscon.dfu -> 64 KB syscon-flash.bin

Exit: 0 done, 2 bad arguments, 3 missing/invalid assets, 4 hash/nonce mismatch, 5 download failed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import struct
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Dict, List, Optional, Sequence

HERE = Path(__file__).resolve().parent.parent
DEST = HERE / "public" / "robot" / "firmware"
PINNED_COMMIT = "3cec4844d24c4719b5b3892e4979494bcd260424"
RAW_BASE = f"https://raw.githubusercontent.com/os-vector/wire-os-victor/{PINNED_COMMIT}"
TIMEOUT_S = 30
DELTA = 0x9E3779B9
MASK = 0xFFFFFFFF
CUBE_NONCE = bytes.fromhex("3d707a86103cef4f832fa8a66420badf")
CUBE_VERSION = b"2018-08-14T17:40"
CUBE_PLAIN_MD5 = "47212dc6a762b2cad010a815cca07425"
CUBE_PLAIN_SHA256 = "c53e7f1edae6bb034cd18782d01b8fb01ec4e1200b68187931060be59db02bee"
SYSCON_FLASH_SHA256 = "cf379f93879875e71e3c3b689368a44db61a5f540554d0a4ff207f4fba13cea7"
FLASH_SIZE = 0x10000
APP_OFF = 0x2000

URLS = {
    "cube.dfu": f"{RAW_BASE}/resources/assets/cubeFirmware/cube.dfu",
    "key.h": f"{RAW_BASE}/robot/cube_firmware/tools/key.h",
    "sysboot.bin": f"{RAW_BASE}/robot/fixture/stm/releases/sysboot.bin",
    "syscon.dfu": f"{RAW_BASE}/robot/fixture/stm/releases/syscon.dfu",
}
EXPECTED_MD5 = {
    "cube.dfu": "b6dbd8b5e34e2d55a62eb21401145942",
    "sysboot.bin": "e1092bc50957c0976f42970cd71c005f",
    "syscon.dfu": "872d8c5330c70236827b18838df61fb3",
}


class FirmwareError(Exception):
    """Raised when a firmware asset fails download or integrity checks."""

    def __init__(self, code: int, msg: str) -> None:
        super().__init__(msg)
        self.code = code


def _mx(z: int, y: int, total: int, p: int, e: int, key: Sequence[int]) -> int:
    assert len(key) == 4 and 0 <= e < 4, "invalid XXTEA state"
    left = (((z >> 5) ^ ((y << 2) & MASK)) + ((y >> 3) ^ ((z << 4) & MASK))) & MASK
    right = ((total ^ y) + (key[(p & 3) ^ e] ^ z)) & MASK
    return left ^ right


def decrypt_xxtea(data: bytes, key: Sequence[int]) -> bytes:
    """Decrypt little-endian 32-bit words with Dialog's XXTEA schedule."""
    if len(data) % 4 or len(data) < 8 or len(key) != 4:
        raise FirmwareError(4, "XXTEA input must be >= 8 bytes and 4-byte aligned")
    v = [int.from_bytes(data[i:i + 4], "little") for i in range(0, len(data), 4)]
    n = len(v)
    rounds = 6 + 52 // n
    total = (rounds * DELTA) & MASK
    y = v[0]
    for _ in range(rounds):
        e = (total >> 2) & 3
        for p in range(n - 1, 0, -1):
            v[p] = (v[p] - _mx(v[p - 1], y, total, p, e, key)) & MASK
            y = v[p]
        v[0] = (v[0] - _mx(v[n - 1], y, total, 0, e, key)) & MASK
        y = v[0]
        total = (total - DELTA) & MASK
    return b"".join(w.to_bytes(4, "little") for w in v)


def parse_key_h(text: str) -> List[int]:
    """Parse 16 comma-separated hex bytes from key.h into 4 little-endian words."""
    vals = [int(c, 16) for c in re.split(r"\s*,\s*", text.strip().rstrip(",")) if c]
    if len(vals) != 16 or not all(0 <= x <= 255 for x in vals):
        raise FirmwareError(4, "key.h must contain 16 byte literals")
    return [int.from_bytes(bytes(vals[i:i + 4]), "little") for i in range(0, 16, 4)]


def open_cube_dfu(raw: bytes, key: Sequence[int]) -> bytes:
    """Verify, decrypt, and strip PKCS#7 padding from stock cube.dfu in memory."""
    if len(raw) < 25 or raw[-1] != 0xFF or (len(raw) - 17) % 4:
        raise FirmwareError(4, "invalid cube.dfu container framing")
    if hashlib.md5(raw).hexdigest() != EXPECTED_MD5["cube.dfu"]:
        raise FirmwareError(4, "cube.dfu MD5 mismatch")
    plain = decrypt_xxtea(raw[16:-1], key)
    pad = plain[-1]
    if not (1 <= pad <= 8 and plain[-pad:] == bytes([pad]) * pad):
        raise FirmwareError(4, "cube.plain.bin PKCS#7 padding check failed")
    img = plain[:-pad]
    if img[0x24:0x34] != CUBE_NONCE or img[:16] != CUBE_VERSION or img[:16] != raw[:16]:
        raise FirmwareError(4, "cube.plain.bin nonce or version check failed")
    if hashlib.md5(img).hexdigest() != CUBE_PLAIN_MD5 or hashlib.sha256(img).hexdigest() != CUBE_PLAIN_SHA256:
        raise FirmwareError(4, "cube.plain.bin hash mismatch")
    return img


def build_syscon_flash(sysboot: bytes, syscon_dfu: bytes) -> bytes:
    """Construct the 64 KB STM32F030 flash image from sysboot.bin and syscon.dfu."""
    if len(sysboot) > APP_OFF or len(syscon_dfu) <= 16:
        raise FirmwareError(4, "invalid sysboot/syscon sizes")
    if hashlib.md5(sysboot).hexdigest() != EXPECTED_MD5["sysboot.bin"]:
        raise FirmwareError(4, "sysboot.bin MD5 mismatch")
    if hashlib.md5(syscon_dfu).hexdigest() != EXPECTED_MD5["syscon.dfu"]:
        raise FirmwareError(4, "syscon.dfu MD5 mismatch")
    body = syscon_dfu[16:]
    if APP_OFF + 0x10 + len(body) > FLASH_SIZE:
        raise FirmwareError(4, "syscon payload overflows 64 KB flash")
    img = bytearray(b"\xff" * FLASH_SIZE)
    img[:len(sysboot)] = sysboot
    img[APP_OFF:APP_OFF + 4] = b"C2MO"
    img[APP_OFF + 0x10:APP_OFF + 0x10 + len(body)] = body
    whiskey = struct.unpack_from("<H", img, APP_OFF + 0x128)[0]
    if whiskey != 0x574B or hashlib.sha256(img).hexdigest() != SYSCON_FLASH_SHA256:
        raise FirmwareError(4, f"syscon image check failed (magic=0x{whiskey:04x})")
    return bytes(img)


def download_inputs() -> Dict[str, bytes]:
    """Download pinned WireOS firmware files from GitHub raw content."""
    assert len(URLS) == 4, "expected 4 firmware URLs"
    out: Dict[str, bytes] = {}
    for name, url in URLS.items():
        try:
            with urllib.request.urlopen(url, timeout=TIMEOUT_S) as resp:
                data = resp.read()
        except (OSError, urllib.error.URLError) as err:
            raise FirmwareError(5, f"download failed for {name}: {err}") from err
        if not data:
            raise FirmwareError(5, f"empty response for {name}")
        out[name] = data
    return out


def load_local_raw(folder: Path) -> Optional[Dict[str, bytes]]:
    """Load raw firmware inputs from folder when present."""
    assert folder.is_dir(), f"missing asset dir: {folder}"
    if not all((folder / n).is_file() for n in URLS):
        return None
    out = {n: (folder / n).read_bytes() for n in URLS}
    assert len(out) == len(URLS), "incomplete raw firmware set"
    return out


def install_prepackaged(folder: Path, dest: Path) -> bool:
    """Copy already-verified cube.plain.bin, syscon-flash.bin, and manifest if present."""
    names = ("cube.plain.bin", "syscon-flash.bin", "firmware_manifest.json")
    if not all((folder / n).is_file() for n in names):
        return False
    manifest = json.loads((folder / "firmware_manifest.json").read_text(encoding="utf-8"))
    cube = (folder / "cube.plain.bin").read_bytes()
    flash = (folder / "syscon-flash.bin").read_bytes()
    if hashlib.md5(cube).hexdigest() != CUBE_PLAIN_MD5 or len(flash) != FLASH_SIZE or flash[APP_OFF:APP_OFF + 4] != b"C2MO":
        raise FirmwareError(4, "prebuilt firmware asset verification failed")
    dest.mkdir(parents=True, exist_ok=True)
    for n in names:
        shutil.copy2(folder / n, dest / n)
    assert manifest.get("source_commit") == PINNED_COMMIT, "source_commit mismatch"
    return True


def write_firmware(inputs: Dict[str, bytes], dest: Path) -> Dict[str, object]:
    """Verify, unpack, and write firmware binaries plus manifest to dest."""
    assert len(inputs) == len(URLS), "incomplete firmware inputs"
    key = parse_key_h(inputs["key.h"].decode("ascii"))
    cube_plain = open_cube_dfu(inputs["cube.dfu"], key)
    syscon_flash = build_syscon_flash(inputs["sysboot.bin"], inputs["syscon.dfu"])
    sp, rv = struct.unpack_from("<2I", syscon_flash, APP_OFF + 0x110)
    dest.mkdir(parents=True, exist_ok=True)
    (dest / "cube.plain.bin").write_bytes(cube_plain)
    (dest / "syscon-flash.bin").write_bytes(syscon_flash)
    manifest = {
        "source_commit": PINNED_COMMIT,
        "cube_version": CUBE_VERSION.decode("ascii"),
        "cube_plain_bytes": len(cube_plain),
        "cube_plain_md5": hashlib.md5(cube_plain).hexdigest(),
        "cube_plain_sha256": hashlib.sha256(cube_plain).hexdigest(),
        "syscon_flash_bytes": len(syscon_flash),
        "syscon_flash_sha256": hashlib.sha256(syscon_flash).hexdigest(),
        "syscon_sp": f"0x{sp:08x}",
        "syscon_reset": f"0x{rv:08x}",
        "syscon_magic": "0x574b",
    }
    (dest / "firmware_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def remove_firmware(dest: Path) -> None:
    """Delete fetched firmware binaries from dest."""
    assert dest.name == "firmware", f"unexpected firmware dir: {dest}"
    shutil.rmtree(dest, ignore_errors=True)
    assert not dest.exists(), f"failed to remove {dest}"


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Fetch, verify, or remove WireOS firmware binaries in public/robot/firmware/."""
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--download", action="store_true", help="download and verify upstream WireOS firmware")
    mode.add_argument("--assets", type=Path, help="copy/build from a local folder holding raw or unpacked files")
    mode.add_argument("--remove", action="store_true", help="remove public/robot/firmware/")
    args = p.parse_args(argv)
    if args.remove:
        remove_firmware(DEST)
        return 0
    try:
        if args.assets is not None:
            if not args.assets.is_dir():
                raise FirmwareError(3, f"missing assets folder: {args.assets}")
            if install_prepackaged(args.assets, DEST):
                print(f"installed preverified firmware to {DEST}")
                return 0
            inputs = load_local_raw(args.assets)
            if inputs is None:
                raise FirmwareError(3, f"missing firmware files in {args.assets}")
        else:
            inputs = download_inputs()
        manifest = write_firmware(inputs, DEST)
    except FirmwareError as exc:
        print(str(exc), file=sys.stderr)
        return exc.code
    print(f"wrote verified firmware to {DEST} (cube={manifest['cube_plain_bytes']}B, syscon={manifest['syscon_flash_bytes']}B)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
