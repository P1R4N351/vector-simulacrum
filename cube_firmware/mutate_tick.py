"""Break tick_avg.s once per arm and show the image check catches it..

    CUBE_STOCK_DFU=... CUBE_KEY=... python3 cube_firmware/mutate_tick.py

tests/mutate.py cannot carry these arms: the image tests need unit 3's file, which a GPU host does not have, so there
they skip and every arm would survive. For each arm: the text must occur exactly once and the file must change;
the mutated tick is assembled, patched into the image and run in the interpreter on the same scenarios as
tests/test_cube_firmware.py. An arm is CAUGHT when the messages differ from the model of the mean app, or the
build refuses it; SURVIVED when they match. A control run of the unmutated file must match first. Exit codes: 0
every arm caught and the control matched, 7 an arm survived, 3 a set-up error.
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path
from typing import List, Tuple

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / "tests"))

from cube_firmware import dfu  # noqa: E402
import test_cube_firmware as tcf  # noqa: E402

ARMS: Tuple[Tuple[str, str, str], ...] = (
    ("drops three low bits, not four", "    asrs  r0, r0, #4", "    asrs  r0, r0, #3"),
    ("no rounding", "    lsrs  r1, r7, #1\n", "    movs  r1, #0\n"),
    ("sums start at 2048 n", "    lsls  r0, r5, #12", "    lsls  r0, r5, #11"),
    ("only two axes averaged", "    adds  r6, #8                 @ r6 + 8 = the z sum",
     "    adds  r6, #4                 @ r6 + 8 = the z sum"),
    ("a tick with no frame is averaged anyway", "    beq   .Lsend", "    beq   .Lmean"),
    ("the limit after a drop is 89", "    movs  r2, #90                @ after a drop", "    movs  r2, #89                @ after a drop"),
    ("the tap threshold is 2881", "    .word -2880", "    .word -2881"),
    ("the tap window lasts 11 frames", "    cmp   r1, #80", "    cmp   r1, #79"),
    ("the tap count is not kept", "    strb  r2, [r4, #17]          @ tap_count++", "    strb  r2, [r4, #18]          @ tap_count++"),
    ("four ticks per message", "    cmp   r0, #3", "    cmp   r0, #4"),
    ("the window opens on 2880 itself", "    ble   .Lcheck", "    blt   .Lcheck"),
    ("the window opens on -2880 itself", "    blt   .Lwindow", "    ble   .Lwindow"),
)
SEEDS = (11, 12, 13)


def check(source: Path, stock: bytes) -> str:
    """MATCH or DIFFER against the model of the mean app, or BUILD-REFUSED."""
    try:
        tick = dfu.assemble(source, HERE / "link.ld")
    except dfu.CheckFailed:
        return "BUILD-REFUSED"
    image = dfu.patch(stock, tcf.VERSION, tick)
    for seed in SEEDS:
        ticks = tcf.scenario(seed, 600) + tcf.boundary()
        try:
            sent, _ = tcf.run_image_tick(image, ticks)
        except Exception:  # a crash, a wild jump, a FIFO left unread: all differ from the model
            return "DIFFER"
        if sent != tcf.model_messages(ticks, "mean"):
            return "DIFFER"
    return "MATCH"


def main() -> int:
    if not tcf.HAVE_IMAGE or not tcf.HAVE_ARM:
        print("SETUP-ERROR: " + tcf.IMAGE_SKIP + ", with the ARM GNU toolchain")
        return 3
    stock = dfu.open_dfu(Path(tcf.STOCK).read_bytes(), dfu.read_key(Path(tcf.KEY)))
    text = (HERE / "tick_avg.s").read_text()
    control = check(HERE / "tick_avg.s", stock)
    print("control (unmutated): %s" % control)
    if control != "MATCH":
        return 3
    survived: List[str] = []
    with tempfile.TemporaryDirectory() as tmp:
        for i, (name, old, new) in enumerate(ARMS, 1):
            if text.count(old) != 1 or old == new:
                print("arm %d %s: SETUP-ERROR (the text occurs %d times)" % (i, name, text.count(old)))
                return 3
            path = Path(tmp) / ("arm%d.s" % i)
            path.write_text(text.replace(old, new))
            verdict = check(path, stock)
            caught = verdict in ("DIFFER", "BUILD-REFUSED")
            print("arm %d %s: %s (%s)" % (i, name, "CAUGHT" if caught else "SURVIVED", verdict))
            if not caught:
                survived.append(name)
    print("result: %d of %d arms caught, %d survived" % (len(ARMS) - len(survived), len(ARMS), len(survived)))
    return 7 if survived else 0


if __name__ == "__main__":
    sys.exit(main())
