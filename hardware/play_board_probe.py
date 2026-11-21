"""Play one probe per armed capture using the configured Vector SDK."""
import argparse
from pathlib import Path

import anki_vector

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--serial", required=True)
args = parser.parse_args()
probe = Path(__file__).with_name("board-probe.wav")
with anki_vector.Robot(serial=args.serial) as robot:
    for label in ("open-before", "40", "60", "80", "open-after"):
        input(f"{label}: position board, arm four-mic recording, then press Enter to play: ")
        robot.audio.stream_wav_file(str(probe), volume=50)
        input(f"Wait for file close, copy this raw take as {label}.wav, then press Enter: ")
