"""Sensor scatter in a held spin, with the IMU chip's filter and without it. Authored-by: CPCS.

    ./run_in_ct.sh evidence/imu-filter-20261120/scatter.py

Needs MuJoCo. Runs measure.py's scatter_in_motion (head 22.4 degrees, spins
at 30, 60 and 100 mm/s, the largest axis's standard deviation over 21 state
messages) on the table as it stands, then with one section, then with no
filter (cutoffs of 1 MHz, one section).
"""

import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import measure  # noqa: E402
from simulacrum import params  # noqa: E402


def main() -> int:
    doc = params.load()
    print("two equal sections (table):")
    measure.scatter_in_motion(doc)
    one = copy.deepcopy(doc)
    one["parameters"]["imu_filter_second_pole"]["value"] = 0.0
    print("one section:")
    measure.scatter_in_motion(one)
    raw = copy.deepcopy(doc)
    for name, value in (("accel_filter_cutoff_hz", 1e6), ("gyro_filter_cutoff_hz", 1e6), ("imu_filter_second_pole", 0.0)):
        raw["parameters"][name]["value"] = value
    print("no filter:")
    measure.scatter_in_motion(raw)
    return 0


if __name__ == "__main__":
    sys.exit(main())
