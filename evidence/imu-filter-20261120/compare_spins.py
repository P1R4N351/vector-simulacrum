"""The IMU chip's filter against both robots' spins. Authored-by: CPCS.

    ./run_in_ct.sh evidence/imu-filter-20261120/compare_spins.py

Needs MuJoCo. Checks the filter's wiring on one robot (first reading, updates
per tick), then replays unit 1's run E spins and unit 3's camera spins on the
table as it stands with three filters: none (cutoffs of 1 MHz, one section,
which passes the raw reading), one section, and two equal sections (the
table's). Prints each spin's steady turn rate and mean |roll| and |pitch|
(rad/s), model beside robot, as simulacrum/spin_fit.py computes them.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulacrum import params, spin_fit  # noqa: E402
from simulacrum.robot import VectorSim  # noqa: E402

VARIANTS = (("none", dict(accel_filter_cutoff_hz=1e6, gyro_filter_cutoff_hz=1e6, imu_filter_second_pole=0.0)),
            ("one section", dict(imu_filter_second_pole=0.0)),
            ("two equal (table)", {}))


def wiring() -> None:
    v = params.values(params.load(model="vector_1"))
    sim = VectorSim(values=v)
    first = sim.states[-1]
    print("first state |accel| %.1f mm/s^2 (gravity %.1f); filter updates after settling %d"
          % (math.sqrt(sum(a * a for a in first.accel_mmps2)), v["gravity_mmps2"], sim._imu_filter.updates))
    sim.step(10)
    print("after 10 ticks: %d filter updates (%d physics steps per tick)" % (sim._imu_filter.updates, sim.d["substeps"]))


def main() -> int:
    wiring()
    for label, path, model in (("Unit 1 run E", spin_fit.FIT_RECORDING, "vector_1"),
                               ("Unit 3 camera", spin_fit.HELD_OUT_RECORDING, "vector_2")):
        base = params.values(params.load(model=model))
        print("%s, inset %.2f mm" % (label, base["tread_contact_inset_mm"]))
        for name, over in VARIANTS:
            for r in spin_fit.compare(path, dict(base, **over)):
                m, b = r["model"], r["robot"]
                print("%-11s %-17s %-13s model rate %.3f roll %.3f pitch %.3f | robot rate %.3f roll %.3f pitch %.3f"
                      " | ratio %.3f" % (label, name, r["label"], m["rate"], m["roll"], m["pitch"],
                                         b["rate"], b["roll"], b["pitch"], r["ratio"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
