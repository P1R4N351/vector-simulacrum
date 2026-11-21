"""Fit tread_contact_inset_mm on unit 1's recorded spins and score unit 3's..

    the GPU host python fit_spins.py          (or ./run_in_ct.sh fit_spins.py)

Needs MuJoCo. Prints one JSON report; see simulacrum/spin_fit.py for what it holds.
"""

from __future__ import annotations

import sys

from simulacrum import spin_fit

if __name__ == "__main__":
    sys.exit(spin_fit.main())
