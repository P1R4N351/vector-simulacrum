"""Collect distinct full-resolution JPEGs from the shelf camera on its host."""
import argparse
import csv
import hashlib
from pathlib import Path
import time
import urllib.request

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('directory', type=Path)
parser.add_argument('--url', required=True)
parser.add_argument('--seconds', type=float, required=True)
args = parser.parse_args()
if not 0 < args.seconds <= 180:
    parser.error('duration must be between 0 and 180 seconds')
args.directory.mkdir(parents=True, exist_ok=False)
deadline = time.monotonic() + args.seconds
previous = None
with (args.directory / 'frames.csv').open('w', newline='') as manifest:
    writer = csv.writer(manifest)
    writer.writerow(['filename', 'request_unix_ns', 'received_unix_ns', 'bytes', 'sha256'])
    while time.monotonic() < deadline:
        start = time.time_ns()
        with urllib.request.urlopen(args.url, timeout=2) as response:
            frame = response.read()
        end = time.time_ns()
        if not (frame.startswith(b'\xff\xd8') and frame.endswith(b'\xff\xd9')):
            raise ValueError('incomplete JPEG')
        digest = hashlib.sha256(frame).hexdigest()
        if digest != previous:
            name = f'{end}.jpg'
            (args.directory / name).write_bytes(frame)
            writer.writerow([name, start, end, len(frame), digest])
            manifest.flush()
            previous = digest
        time.sleep(.010)
