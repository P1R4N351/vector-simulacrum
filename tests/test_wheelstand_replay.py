"""Replays of the two physical wheelstands, W4 and W8 (evidence/unit3-wheelstand/reproduction).

Needs the printed-fork fixture XML built by evidence/unit3-printed-fork-contact/build_fixture.py from
forks7.stl (not in this repository); point WHEELSTAND_FORK_FIXTURE at it. Without it these tests skip as
an optional input. Each replay takes about 9 s of CPU.

Four kinds of check:
  - determinism: the same inputs give the same trace, bit for bit;
  - the rear tip-support fix: fixed supports cap the pitch below 48 degrees, driven ones do not;
  - a ratchet on the current match: the medians over the four onset cases must not get worse than
    the values recorded in reproduction/README.md by more than the stated margins;
  - the physical match itself, which the model does not yet reach: an expected failure, so the day it
    passes the suite turns red and the tolerances move into the ratchet.
"""

from __future__ import annotations

import importlib.util
import os
import unittest
from pathlib import Path

import _common

ROOT = _common.ROOT
FIXTURE = os.environ.get("WHEELSTAND_FORK_FIXTURE", "")
REPLAY = ROOT / "evidence/unit3-wheelstand/reproduction/replay.py"
SKIP = f"{_common.OPTIONAL_SKIP_PREFIX}set WHEELSTAND_FORK_FIXTURE to the printed-fork fixture XML"

# The reference configuration and the medians it reached when this test was written (README, ablation.csv).
REFERENCE = dict(driven_tips=1, lift="servo:60")
RATCHET = {"W4": dict(pitch_rmse_deg=18.40, peak_err_deg=13.03),
           "W8": dict(pitch_rmse_deg=29.97, peak_err_deg=22.91)}
MARGIN = dict(pitch_rmse_deg=2.0, peak_err_deg=3.0)
# Physical tolerances (README): the two recordings agree on peak pitch within 0.25 deg and on the hold
# within 0.1 deg; time to peak carries the 65 ms onset ambiguity plus state-receipt jitter.
TOL = dict(peak_pitch_deg=3.0, t_peak_s=0.08, hold_pitch_deg=3.0)


def _load_replay():
    spec = importlib.util.spec_from_file_location("wheelstand_replay", REPLAY)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _cases(rp, trial, driven_tips, lift, which=None):
    """Run the onset cases of one trial (all four, or the one at index `which`); returns (scores, traces)."""
    from simulacrum import params
    v = params.values(params.load(model="vector_2"), {"tip_supports_driven": driven_tips})
    m = rp.build(v, FIXTURE)
    m.opt.timestep = v["sim_timestep_ms"] * 1e-3 / 4
    tr = rp.Trial(trial)
    cases = [(s + tr.shift, o) for s in (3.5, 3.8) for o in (tr.early, tr.late)]
    if which is not None:
        cases = cases[which:which + 1]
    scores, traces = [], []
    for start, onset in cases:
        trace, packets, taps, _ = rp.run(m, v, tr, start, onset, lift, 17)
        scores.append(rp.score(tr, trace, packets, taps, onset))
        traces.append(trace)
    return tr, scores, traces


def _median(values):
    s = sorted(values)
    n = len(s)
    assert n > 0
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class WheelstandReplay(unittest.TestCase):
    rp = None

    def setUp(self) -> None:
        if not FIXTURE or not Path(FIXTURE).is_file():
            self.skipTest(SKIP)
        if WheelstandReplay.rp is None:
            WheelstandReplay.rp = _load_replay()

    def test_same_inputs_give_the_same_trace(self) -> None:
        _, _, first = _cases(self.rp, "W4", 1, "servo:60", which=1)
        _, _, second = _cases(self.rp, "W4", 1, "servo:60", which=1)
        self.assertEqual(first[0].tobytes(), second[0].tobytes())

    def test_fixed_tip_supports_cap_the_pitch_and_driven_ones_do_not(self) -> None:
        for trial in ("W4", "W8"):
            _, fixed, _ = _cases(self.rp, trial, 0, "servo:100", which=1)
            _, driven, _ = _cases(self.rp, trial, 1, "servo:100", which=1)
            self.assertLess(fixed[0]["peak_pitch_deg"], 48.0, trial)
            self.assertGreater(driven[0]["peak_pitch_deg"], 50.0, trial)

    def test_the_match_does_not_get_worse(self) -> None:
        for trial in ("W4", "W8"):
            tr, scores, _ = _cases(self.rp, trial, REFERENCE["driven_tips"], REFERENCE["lift"])
            physical = tr.physical()
            rmse = _median([s["pitch_rmse_deg"] for s in scores])
            peak_err = _median([abs(s["peak_pitch_deg"] - physical["peak_pitch_deg"]) for s in scores])
            self.assertLessEqual(rmse, RATCHET[trial]["pitch_rmse_deg"] + MARGIN["pitch_rmse_deg"], trial)
            self.assertLessEqual(peak_err, RATCHET[trial]["peak_err_deg"] + MARGIN["peak_err_deg"], trial)

    @unittest.expectedFailure
    def test_the_physical_wheelstands_are_matched(self) -> None:
        for trial in ("W4", "W8"):
            tr, scores, _ = _cases(self.rp, trial, REFERENCE["driven_tips"], REFERENCE["lift"])
            physical = tr.physical()
            for key, tol in TOL.items():
                sim = _median([s[key] for s in scores])
                self.assertLessEqual(abs(sim - physical[key]), tol, f"{trial} {key}: {sim:.3f} vs {physical[key]:.3f}")


if __name__ == "__main__":
    unittest.main()
