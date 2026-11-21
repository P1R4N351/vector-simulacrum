"""The cube app with the averaging tick (cube_firmware/), checked against the model before it goes on a cube.

Authored-by: CPCS. The image tests need unit 3's cube.dfu and the key file, given by CUBE_STOCK_DFU and CUBE_KEY;
neither is in this repository (the app is Anki's and the key is theirs), so without them those tests skip and
say so. They also need the ARM GNU toolchain to build the new tick.

How they check: the Thumb interpreter (cube_firmware/thumb.py) runs the shipped tick and the shipped divide
straight from unit 3's image. Agreeing with the model of the shipped app (simulacrum/cube.py, "last") checks the
interpreter and that model against each other, since one comes from the binary and the other from the C
source. Then the new image's tick, exactly the bytes to be flashed, must agree with the model of the new app
("mean"). The C version of the tick, built for this machine, must agree too.
"""

from __future__ import annotations

import os
import random
import shutil
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import _common
from simulacrum import cube
from cube_firmware import dfu
from cube_firmware.thumb import Thumb

STOCK = os.environ.get("CUBE_STOCK_DFU", "")
KEY = os.environ.get("CUBE_KEY", "")
HAVE_IMAGE = bool(STOCK and KEY and Path(STOCK).is_file() and Path(KEY).is_file())
HAVE_ARM = all(shutil.which(t) for t in dfu.TOOLS)
CC = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
IMAGE_SKIP = _common.IMAGE_SKIP_REASON
VERSION = b"cpcs-avg1-261121"
SPI_READ8, SPI_READ, UDIV, SEND_HOOK = 0x20006350, 0x200062f0, 0x200058ba, 0x20009800
STACK_TOP, STACK_BYTES = 0x20009000, 256
STATE_AT, STATE_END = 0x20006430, 0x20006454  # the tick's state block and the message after it

Frames = List[List[int]]


def scenario(seed: int, ticks: int) -> List[Frames]:
    """Ticks of FIFO frames: counts from none to a full FIFO, values to both ends of int16 with the four low
    bits filled at random, z spikes up and drops down that open tap windows, and quiet stretches."""
    rng = random.Random(seed)
    out: List[Frames] = []
    for k in range(ticks):
        n = rng.choice((0, 1, 2, 3, 4, 5, 5, 5, 5, 6, 7, 12, 31, 32))
        base_z = 8192 + (3000 if k % 29 == 3 else 0) - (3300 if k % 41 == 7 else 0)
        frames = []
        for i in range(n):
            wide = rng.random() < 0.05
            frames.append([rng.randrange(-32768, 32768) if wide else rng.randrange(-300, 300) * 16 + rng.randrange(16),
                           rng.randrange(-32768, 32768),
                           base_z + rng.randrange(-60, 60) + (2950 if (k % 17 == 2 and i == 1) else 0)])
        out.append(frames)
    return out


def boundary() -> List[Frames]:
    """Hand-built ticks on each tap rule's edge (accel.c): diffs of exactly +2880 and -2880 (no window, each
    followed by what would tap inside one), +2881 and -2881 (a window), a fall of -2880 and of -2881 in a positive window, a rise of 89 and of 90 in a
    negative one, and a fall in a window's tenth frame and in its eleventh. Five frames a tick, x and y fixed,
    and 100 quiet frames between cases so every window closes."""
    quiet = [0] * 100
    cases = ([2880] + quiet + [-2880] + quiet,                 # neither edge opens a window
             [2880, -2881] + [90] * 10 + quiet,                # had +2880 opened one, the fall would tap
             [-2880, 89] + [90] * 9 + quiet,                   # had -2880 opened one, the 89 would tap
             [2881, -2881] + quiet,                            # a window and a tap
             [2881, -2880] + quiet + [-1] + quiet,             # a window, the fall is one short
             [-2881, 89] + [90] * 9 + quiet,                   # a drop, then a rise below 90: a tap
             [-2881] + [90] * 10 + quiet,                      # a drop, then rises of 90: none
             [2881] + [0] * 9 + [-2881] + quiet,               # the fall in the window's tenth frame: a tap
             [2881] + [0] * 10 + [-2881] + quiet)              # in its eleventh: none
    z, zs = 8192, [8192] * 100
    for case in cases:
        for diff in case:
            z += diff
            zs.append(z)
        z = 8192
        zs.append(z)
        zs.extend([z] * 99)
    zs += [8192] * (-len(zs) % 5)
    return [[[160, -320, v] for v in zs[i:i + 5]] for i in range(0, len(zs), 5)]


def model_messages(ticks: Sequence[Frames], app: str) -> List[bytes]:
    model = cube.CubeApp(app)
    for i, frames in enumerate(ticks):
        model.tick(i * 0.01, frames)
    return [data for _, data in model.messages]


def run_image_tick(image: bytes, ticks: Sequence[Frames]) -> Tuple[List[bytes], int]:
    """The image's tick at 0x20005db4, once per tick, with a stand-in FIFO behind the shipped SPI routines.
    Returns the messages sent and the deepest the stack went, in bytes."""
    m = Thumb()
    m.load(dfu.BASE, image)
    # The tick may read the image and its stack, and write only its state, the message and its stack.
    m.readable = [(dfu.BASE, dfu.BASE + dfu.REGION), (STACK_TOP - STACK_BYTES, STACK_TOP)]
    m.writable = [(STATE_AT, STATE_END), (STACK_TOP - STACK_BYTES, STACK_TOP)]
    sent: List[bytes] = []
    fifo: Frames = []
    deepest = [STACK_TOP]

    def spi_read8(t: Thumb) -> None:
        assert t.r[0] == 0x0E, "the tick reads FIFO_STATUS and nothing else"
        deepest[0] = min(deepest[0], t.r[13])
        t.r[0] = len(fifo)

    def spi_read(t: Thumb) -> None:
        assert t.r[0] == 0x3F and t.r[1] == 6, "the tick reads whole six-byte FIFO frames"
        deepest[0] = min(deepest[0], t.r[13])
        t.load(t.r[2], struct.pack("<3h", *fifo.pop(0)))

    def send(t: Thumb) -> None:
        assert (t.r[0], t.r[1]) == (20, STATE_AT + 16), "the tick sends the 20-byte message"
        sent.append(bytes(t.read(t.r[1] + i, 1) for i in range(t.r[0])))

    m.hooks = {SPI_READ8: spi_read8, SPI_READ: spi_read, SEND_HOOK: send}
    m.writable.append((0x20005820, 0x20005824))
    m.write(0x20005820, 4, SEND_HOOK | 1)  # ApplicationMap.BLE_Send, which the bootloader fills in
    m.writable.pop()
    for frames in ticks:
        fifo[:] = [list(f) for f in frames]
        m.call(0x20005db4, [], STACK_TOP)
        assert not fifo, "the tick must empty the FIFO"
    return sent, STACK_TOP - deepest[0]


def built() -> Tuple[bytes, bytes, bytes]:
    return dfu.build(Path(STOCK).read_bytes(), dfu.read_key(Path(KEY)), VERSION)


@unittest.skipUnless(HAVE_IMAGE, IMAGE_SKIP)
class TheShippedApp(unittest.TestCase):
    def test_xxtea_reads_unit3s_file_and_writes_it_back_byte_for_byte(self) -> None:
        key, raw = dfu.read_key(Path(KEY)), Path(STOCK).read_bytes()
        image = dfu.open_dfu(raw, key)
        self.assertEqual(image[:16], dfu.STOCK_VERSION)
        self.assertEqual(image[dfu.NONCE_AT:dfu.NONCE_AT + 16], dfu.NONCE)
        self.assertEqual(dfu.make_dfu(image, key), raw)
        wrong = [key[0] ^ 1] + key[1:]
        with self.assertRaises(dfu.CheckFailed):
            dfu.open_dfu(raw, wrong)

    def test_the_shipped_divide_at_0x200058ba_divides_unsigned(self) -> None:
        image = dfu.open_dfu(Path(STOCK).read_bytes(), dfu.read_key(Path(KEY)))
        m = Thumb()
        m.load(dfu.BASE, image)
        rng = random.Random(3)
        for _ in range(400):
            n, d = rng.randrange(0, 1 << 32), rng.randrange(1, 1 << rng.choice((2, 6, 16, 31)))
            self.assertEqual((m.call(UDIV, [n, d], STACK_TOP), m.r[1]), divmod(n, d), (n, d))

    def test_the_shipped_tick_matches_the_model_of_the_shipped_app(self) -> None:
        image = dfu.open_dfu(Path(STOCK).read_bytes(), dfu.read_key(Path(KEY)))
        ticks = scenario(11, 600) + boundary()
        sent, _ = run_image_tick(image, ticks)
        want = model_messages(ticks, "last")
        self.assertEqual(len(sent), len(ticks) // 3)
        self.assertEqual(sent, want)
        self.assertGreater(sent[-1][1], 5, "the scenario must make taps")


@unittest.skipUnless(HAVE_IMAGE and HAVE_ARM, IMAGE_SKIP + ", with the ARM GNU toolchain")
class TheNewApp(unittest.TestCase):
    def test_the_new_tick_matches_the_model_of_the_mean_app(self) -> None:
        _, image, _ = built()
        for seed in (11, 12, 13):
            ticks = scenario(seed, 600) + boundary()
            sent, _ = run_image_tick(image, ticks)
            self.assertEqual(sent, model_messages(ticks, "mean"), "seed %d" % seed)
            self.assertNotEqual(sent, model_messages(ticks, "last"))

    def test_its_taps_are_the_shipped_taps(self) -> None:
        stock, image, _ = built()
        ticks = scenario(14, 900) + boundary()
        new, _ = run_image_tick(image, ticks)
        old, _ = run_image_tick(stock, ticks)
        self.assertEqual([m[1] for m in new], [m[1] for m in old])
        self.assertGreater(new[-1][1], 5)

    def test_the_file_changes_only_the_version_and_the_tick_and_the_robot_would_accept_it(self) -> None:
        stock, image, raw = built()
        key = dfu.read_key(Path(KEY))
        self.assertEqual(dfu.open_dfu(raw, key), image)
        self.assertEqual(raw[:16], VERSION)
        self.assertEqual((len(raw), len(image)), (len(Path(STOCK).read_bytes()), len(stock)))
        for start, end in dfu.changed_ranges(stock, image):
            self.assertTrue(end <= dfu.BASE + 16 or (dfu.TICK_AT <= start and end <= dfu.TICK_END), (start, end))
        # The robot sends from byte 16, 20 per write; only the last may be a length the bootloader ends on.
        lengths = [len(raw[i:i + 20]) for i in range(16, len(raw), 20)]
        self.assertEqual([n % 4 != 0 for n in lengths], [False] * (len(lengths) - 1) + [True])

    def test_it_needs_little_more_stack_than_the_shipped_tick(self) -> None:
        stock, image, _ = built()
        ticks = scenario(15, 30)
        _, old = run_image_tick(stock, ticks)
        _, new = run_image_tick(image, ticks)
        self.assertLessEqual(new - old, 16, "stack at the SPI calls: shipped %d bytes, new %d" % (old, new))


@unittest.skipUnless(CC is not None, _common.CC_SKIP_REASON)
class TheCTick(unittest.TestCase):
    def test_the_c_tick_built_here_matches_the_model_of_the_mean_app(self) -> None:
        root = Path(__file__).resolve().parent
        with tempfile.TemporaryDirectory() as tmp:
            exe = Path(tmp) / "harness"
            done = subprocess.run([str(CC), "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                                   str(root / "cube_tick_harness.c"), "-o", str(exe)],
                                  capture_output=True, text=True, timeout=120)
            self.assertEqual(done.returncode, 0, done.stderr[:800])
            ticks = scenario(16, 600) + boundary()
            text = "".join(" ".join([str(len(f))] + [str(v) for frame in f for v in frame]) + "\n" for f in ticks)
            run = subprocess.run([str(exe)], input=text, capture_output=True, text=True, timeout=120)
        self.assertEqual(run.returncode, 0, run.stderr[:400])
        got = [bytes.fromhex(line) for line in run.stdout.split()]
        self.assertEqual(got, model_messages(ticks, "mean"))


class TheBoundaryCases(unittest.TestCase):
    def test_the_model_counts_the_taps_each_edge_case_was_built_for(self) -> None:
        counter = cube.TapCounter()
        counts = []
        for frames in boundary():
            for f in frames:
                counter.feed(f[2])
            counts.append(counter.count)
        steps = sorted(set(counts))
        self.assertEqual(steps, [0, 1, 2, 3], "the start window, then one tap each in cases 2, 4 and 6")


class TheModelsMean(unittest.TestCase):
    def test_the_mean_drops_the_low_bits_and_rounds_halves_up(self) -> None:
        self.assertEqual(cube.mean_counts([8192 + 15]), 8192)
        self.assertEqual(cube.mean_counts([16, 32]), 24)               # 1.5 counts of 12 bits
        self.assertEqual(cube.mean_counts([16, 0]), 8)                 # half a count
        self.assertEqual(cube.mean_counts([-16, 0]), -8)               # halves round up: -0.5 -> 0 would be wrong here
        self.assertEqual(cube.mean_counts([16, 16, 0]), 11)            # 16 x 2/3 = 10.67
        self.assertEqual(cube.mean_counts([-32768] * 32), -32768)
        self.assertEqual(cube.mean_counts([32767] * 32), 32752)


if __name__ == "__main__":
    unittest.main()
