"""Replaying the real robot's recording through the model. Needs MuJoCo..

The scores are the mean error of a group of segments as a fraction of each
segment's own scale (command speed, no-slip turn rate, joint range). The
bounds here are a little above what the table's values give today, so a
change that makes the model fit the robot worse fails.
"""

from __future__ import annotations

import unittest
from pathlib import Path

import _common
from simulacrum import params, recording

if _common.HAVE_MUJOCO:
    from simulacrum import replay

BEFORE_THE_FIT = {
    "wheel_stall_torque_mNm": 25.0, "wheel_armature_gcm2": 1000.0, "tread_noload_speed_mmps": 319.5,
    "tread_friction_power_fraction": 0.057, "command_delay_ms": 30.0, "servo_command_delay_ms": 30.0,
    "head_speed_max_radps": 10.0, "lift_speed_max_radps": 10.0,
}

CONSTANT_RATE = {"head_accel_default_radps2": 1.0e6, "lift_accel_default_radps2": 1.0e6, "head_speed_max_radps": 1.75,
                 "lift_speed_max_radps": 2.8, "servo_command_delay_ms": 225.0}
SECOND = Path(__file__).resolve().parent.parent / "evidence" / "hw-rec-20261118" / "run-a-rest-headlift.jsonl"
SPINS = SECOND.parent / "run-e-spins.jsonl"


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Replay(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.v = params.values(params.load())
        cls.fit = replay.compare(cls.v, replay.FIT_WHEELS)
        cls.hold = replay.compare(cls.v, replay.HOLD_WHEELS)

    def test_fit_and_hold_out_segments_do_not_overlap(self) -> None:
        self.assertEqual(set(replay.FIT_WHEELS) & set(replay.HOLD_WHEELS), set())
        self.assertEqual(set(replay.FIT_SERVOS) & set(replay.HOLD_SERVOS), set())
        self.assertEqual(len(replay.FIT_WHEELS) + len(replay.HOLD_WHEELS), 10)

    def test_every_segment_is_compared_on_its_recorded_samples(self) -> None:
        counts = {r["label"]: r["n"] for r in self.fit + self.hold}
        self.assertEqual(counts["spin_ccw_30"], 58)
        self.assertEqual(counts["forward_80"], 22)
        self.assertEqual(sum(counts.values()), 408)

    def test_wheel_segments_the_fit_used_match_the_robot(self) -> None:
        self.assertLess(replay.wheel_score(self.fit, self.v), 0.19)
        for result in self.fit:
            self.assertLess(result["wheel_rmse_mmps"] / result["command_mmps"], 0.22, result["label"])

    def test_wheel_segments_held_out_match_the_robot_as_well(self) -> None:
        self.assertLess(replay.wheel_score(self.hold, self.v), 0.17)
        for result in self.hold:
            self.assertLess(result["wheel_rmse_mmps"] / result["command_mmps"], 0.15, result["label"])

    def test_the_fitted_values_beat_the_ones_before_the_fit_on_held_out_segments(self) -> None:
        old = dict(self.v, **BEFORE_THE_FIT)
        before = replay.wheel_score(replay.compare(old, replay.HOLD_WHEELS), old)
        self.assertGreater(before, replay.wheel_score(self.hold, self.v) + 0.15)

    def test_head_and_lift_match_the_robot_on_fit_and_held_out_moves(self) -> None:
        fit = replay.compare(self.v, replay.FIT_SERVOS)
        hold = replay.compare(self.v, replay.HOLD_SERVOS)
        self.assertLess(replay.servo_score(fit, self.v), 0.035)
        held = replay.servo_score(hold, self.v)
        self.assertLess(held, 0.04)
        # With the ramp, the 30 ms delay from before any fit scores 0.080 here; the fitted delay 0.035.
        old = dict(self.v, **BEFORE_THE_FIT)
        self.assertGreater(replay.servo_score(replay.compare(old, replay.HOLD_SERVOS), old), held + 0.03)

    def test_head_and_lift_match_the_second_recording_which_no_fit_saw(self) -> None:
        """REC2 run A, 2026-11-18: 8 head and 7 lift moves, the full-range ones longer than any in REC."""
        _, commands = recording.load(SECOND)
        labels = [k for k, seg in recording.segments(commands, self.v).items() if seg.kind in ("head", "lift")]
        self.assertEqual(len(labels), 15)
        now = replay.servo_score(replay.compare(self.v, labels, SECOND), self.v)
        self.assertLess(now, 0.04)
        # The constant-rate head and lift this replaced (no ramp, caps 1.75 and 2.8 rad/s, 225 ms) scores 0.060.
        flat = dict(self.v, **CONSTANT_RATE)
        self.assertGreater(replay.servo_score(replay.compare(flat, labels, SECOND), flat), now + 0.02)

    def test_the_fast_spins_of_the_second_recording_replay(self) -> None:
        """REC2 run E: four spins at 150 and 220 mm/s; the wheel speeds match, the turn rate is short (README)."""
        states, commands = recording.load(SPINS)
        segs = {k: seg for k, seg in recording.segments(commands, self.v).items() if seg.kind == "wheels"}
        self.assertEqual(sorted(segs), ["spin_ccw_150", "spin_ccw_220", "spin_cw_150", "spin_cw_220"])
        for label, seg in segs.items():
            result = replay.replay(seg, self.v, states)
            self.assertLess(result["wheel_rmse_mmps"] / result["command_mmps"], 0.12, label)

    def test_replay_gives_the_same_result_twice(self) -> None:
        self.assertEqual(replay.compare(self.v, ("spin_cw_60",)), replay.compare(self.v, ("spin_cw_60",)))


if __name__ == "__main__":
    unittest.main()
