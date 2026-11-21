"""The ranking of the assumed parameters. The arithmetic needs no MuJoCo; the runs do.."""

from __future__ import annotations

import unittest

import _common
import rank_assumed
from simulacrum import params

SIGMA_F, SIGMA_T = 40.0, 30.0
NAMES = ["left_wheel_speed", "right_wheel_speed", "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z",
         "cliff_detected", "prox_distance", "prox_found_object", "head_angle", "lift_height"]


def _open(forward: float, turn: float, wheel_obs: float, fallen: bool = False) -> dict:
    """A made-up open_loop result: every scenario has two steps with the same motion and observation."""
    obs = tuple([wheel_obs] + [0.0] * (rank_assumed.REPORTED - 1))
    runs = {name: {"motion": [(forward, turn)] * 2, "obs": [obs] * 2} for name, _, _, _ in rank_assumed.OPEN_LOOP}
    edges = {name: {"fallen": fallen, "reactions": 0, "past_edge_mm": 5.0, "cliff_step": 22}
             for name, _, _ in rank_assumed.EDGES}
    return {"runs": runs, "edges": edges, "names": list(NAMES)}


def _row(name: str, bound: str, reward: float, opened: dict, falls: int = 0) -> dict:
    return {"name": name, "bound": bound, "open": opened,
            "closed": {"no_slip": {"reward_per_step": reward, "falls": falls}}}


class Arithmetic(unittest.TestCase):
    def test_a_variant_moves_one_assumed_entry_to_its_bound_and_nothing_else(self) -> None:
        doc = params.load()
        low = rank_assumed.variant(doc, "tread_friction", "low")
        entry = doc["parameters"]["tread_friction"]
        self.assertEqual(low["parameters"]["tread_friction"]["value"], entry["low"])
        self.assertNotEqual(entry["value"], entry["low"])
        changed = [name for name in doc["parameters"] if low["parameters"][name] != doc["parameters"][name]]
        self.assertEqual(changed, ["tread_friction"])
        self.assertEqual(params.validate(low), [])

    def test_only_assumed_entries_can_be_varied(self) -> None:
        doc = params.load()
        self.assertNotIn("wheel_stall_torque_mNm", rank_assumed.assumed_names(doc))
        with self.assertRaises(AssertionError):
            rank_assumed.variant(doc, "wheel_stall_torque_mNm", "low")
        counts = params.summary(doc)
        self.assertEqual(len(rank_assumed.assumed_names(doc)), counts["assumed"] + counts["chosen"])

    def test_change_is_measured_in_the_rewards_own_widths(self) -> None:
        base = _open(100.0, 0.0, 0.5)
        same = rank_assumed.open_loop_change(base, _open(100.0, 0.0, 0.5), SIGMA_F, SIGMA_T)
        self.assertEqual((same["motion"], same["obs_rms"], same["edge_flips"], same["edges"]), (0.0, 0.0, 0, []))
        faster = rank_assumed.open_loop_change(base, _open(120.0, 0.0, 0.5), SIGMA_F, SIGMA_T)
        self.assertAlmostEqual(faster["motion"], 20.0 / SIGMA_F, delta=1e-9)
        turning = rank_assumed.open_loop_change(base, _open(100.0, 45.0, 0.5), SIGMA_F, SIGMA_T)
        self.assertAlmostEqual(turning["motion"], 45.0 / SIGMA_T, delta=1e-9)
        seen = rank_assumed.open_loop_change(base, _open(100.0, 0.0, 0.6), SIGMA_F, SIGMA_T)
        self.assertAlmostEqual(seen["obs_rms"], 0.1, delta=1e-4)
        self.assertEqual(seen["obs_channel"], "left_wheel_speed")
        fell = rank_assumed.open_loop_change(base, _open(100.0, 0.0, 0.5, fallen=True), SIGMA_F, SIGMA_T)
        self.assertEqual(fell["edge_flips"], len(rank_assumed.EDGES))

    def test_order_is_by_the_larger_reward_change_of_the_two_bounds(self) -> None:
        base = _row("", "", 1.0, _open(100.0, 0.0, 0.5))
        rows = [_row("small", "low", 1.01, _open(100.0, 0.0, 0.5)), _row("small", "high", 0.98, _open(100.0, 0.0, 0.5)),
                _row("big", "low", 1.0, _open(100.0, 0.0, 0.5)), _row("big", "high", 0.7, _open(130.0, 0.0, 0.5), falls=3),
                _row("none", "low", 1.0, _open(100.0, 0.0, 0.5)), _row("none", "high", 1.0, _open(100.0, 0.0, 0.5))]
        ordered = rank_assumed.rank(base, rows, SIGMA_F, SIGMA_T)
        self.assertEqual([entry["name"] for entry in ordered], ["big", "small", "none"])
        self.assertAlmostEqual(ordered[0]["reward"], 0.3, delta=1e-9)
        self.assertAlmostEqual(ordered[1]["reward"], 0.02, delta=1e-9)
        self.assertEqual(ordered[0]["detail"]["high"]["d_falls"], {"no_slip": 3})
        self.assertEqual((ordered[2]["reward"], ordered[2]["motion"]), (0.0, 0.0))


    def test_an_entry_clears_the_floor_only_by_beating_the_null_runs(self) -> None:
        base = _row("", "", 1.0, _open(100.0, 0.0, 0.5))
        nulls = [_row("total_mass_g", "null0", 1.02, _open(102.0, 0.0, 0.5)),
                 _row("total_mass_g", "null1", 0.97, _open(100.0, 3.0, 0.5))]
        floors = rank_assumed.floor(base, nulls, SIGMA_F, SIGMA_T)
        self.assertEqual(floors["runs"], 2)
        self.assertAlmostEqual(floors["reward"]["no_slip"], 0.03, delta=1e-9)
        self.assertAlmostEqual(floors["motion"], 0.1, delta=1e-9)
        rows = [_row("under", "low", 1.02, _open(101.0, 0.0, 0.5)), _row("under", "high", 0.98, _open(100.0, 0.0, 0.5)),
                _row("over", "low", 1.0, _open(100.0, 0.0, 0.5)), _row("over", "high", 0.9, _open(100.0, 0.0, 0.5)),
                _row("moves", "low", 1.0, _open(100.0, 0.0, 0.5)), _row("moves", "high", 1.0, _open(110.0, 0.0, 0.5))]
        ranked = rank_assumed.rank(base, rows, SIGMA_F, SIGMA_T, floors)
        self.assertEqual({e["name"]: e["clears_floor"] for e in ranked}, {"under": False, "over": True, "moves": False})
        self.assertEqual({e["name"]: e["motion_clears_floor"] for e in ranked},
                         {"under": False, "over": False, "moves": True})

    def test_a_nudge_changes_the_mass_by_under_a_gram_and_nothing_else(self) -> None:
        doc = params.load()
        nudged = rank_assumed.nudged(doc, 2)
        ratio = nudged["parameters"]["total_mass_g"]["value"] / doc["parameters"]["total_mass_g"]["value"]
        self.assertAlmostEqual(ratio, 1.003, delta=1e-9)
        self.assertLess(nudged["parameters"]["total_mass_g"]["value"] - doc["parameters"]["total_mass_g"]["value"], 1.0)
        entry = doc["parameters"]["total_mass_g"]
        last = rank_assumed.nudged(doc, 5)["parameters"]["total_mass_g"]["value"]
        self.assertTrue(entry["low"] < last < entry["high"])
        changed = [name for name in doc["parameters"] if nudged["parameters"][name] != doc["parameters"][name]]
        self.assertEqual(changed, ["total_mass_g"])


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Runs(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.doc = params.load()
        cls.base = rank_assumed.open_loop(cls.doc)

    def test_the_same_table_twice_gives_no_change(self) -> None:
        again = rank_assumed.open_loop(self.doc)
        change = rank_assumed.open_loop_change(self.base, again, SIGMA_F, SIGMA_T)
        self.assertEqual((change["motion"], change["obs_rms"], change["accel_rms"], change["edge_flips"]), (0.0, 0.0, 0.0, 0))
        self.assertEqual(self.base["names"], NAMES)
        self.assertEqual(change["steps"], sum(len(run["motion"]) for run in self.base["runs"].values()))

    def test_an_entry_the_task_never_reads_changes_nothing_and_slippery_treads_change_the_motion(self) -> None:
        unused = rank_assumed.open_loop(rank_assumed.variant(self.doc, "heading_hold_gain_mmps_per_rad", "low"))
        self.assertEqual(rank_assumed.open_loop_change(self.base, unused, SIGMA_F, SIGMA_T)["motion"], 0.0)
        slippery = rank_assumed.open_loop(rank_assumed.variant(self.doc, "tread_friction", "low"))
        self.assertGreater(rank_assumed.open_loop_change(self.base, slippery, SIGMA_F, SIGMA_T)["motion"], 0.01)

    def test_edges_end_as_the_control_level_says(self) -> None:
        edges = self.base["edges"]
        self.assertTrue(edges["override 100"]["fallen"] and edges["override 100"]["reactions"] == 0)
        self.assertTrue(not edges["default 100"]["fallen"] and edges["default 100"]["reactions"] == 1)
        self.assertIsNotNone(edges["default 100"]["cliff_step"])

    def test_closed_loop_repeats_exactly_and_counts_every_episode(self) -> None:
        first = rank_assumed.closed_loop(self.doc, 2, 5000, None)
        second = rank_assumed.closed_loop(self.doc, 2, 5000, None)
        self.assertEqual(first, second)
        row = first["no_slip"]
        self.assertEqual(row["falls"] + row["reactions"] + row["time_limits"], 2)
        self.assertGreater(row["steps"], 2)


if __name__ == "__main__":
    unittest.main()
