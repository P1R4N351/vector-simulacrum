"""The spin fit's two recordings on a fine grid of contact insets. Authored-by: CPCS.

    ./run_in_ct.sh evidence/imu-filter-20261120/inset_scan.py

Needs MuJoCo. fit_spins.py steps the inset by 1 mm. This replays unit 1's
run E spins (1.0 table) and unit 3's camera spins (2.0 table) at 21 insets
from 2.90 to 3.30 mm, 0.02 mm apart, to show how smoothly the model's turn
rate follows the inset between the grid points. Prints, per inset, each
recording's mean rate ratio (model over robot), the four spins' ratios, and
the model's turn centre x for unit 3.
"""

import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from simulacrum import params, spin_fit  # noqa: E402

STEPS = 21


def main() -> int:
    v1 = params.values(params.load(model="vector_1"))
    v2 = params.values(params.load(model="vector_2"))
    for k in range(STEPS):
        inset = round(2.9 + 0.02 * k, 2)
        fit = spin_fit.compare(spin_fit.FIT_RECORDING, dict(v1, tread_contact_inset_mm=inset))
        held = spin_fit.compare(spin_fit.HELD_OUT_RECORDING, dict(v2, tread_contact_inset_mm=inset))
        print("inset %.2f | unit 1 mean %.4f spins %s | unit 3 mean %.4f spins %s centre %s" % (
            inset, statistics.mean(r["ratio"] for r in fit), " ".join("%.3f" % r["ratio"] for r in fit),
            statistics.mean(r["ratio"] for r in held), " ".join("%.3f" % r["ratio"] for r in held),
            " ".join("%.1f" % r["model"]["centre_x_mm"] for r in held)), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
