"""Tests for NavMap2D, optical marker detection, exploration, and self-docking."""

from __future__ import annotations

import unittest

import _common
from simulacrum import autonomy


class NavMapUnit(unittest.TestCase):
    def test_cliff_is_sticky_and_segment_clear_detects_cube(self) -> None:
        grid = autonomy.NavMap2D()
        grid.record_cube({"x": 100.0, "y": 0.0}, tick=5)
        self.assertFalse(grid.segment_clear(0.0, 0.0, 180.0, 0.0))
        self.assertTrue(grid.segment_clear(0.0, 100.0, 180.0, 100.0))
        grid.set_cell(20.0, 20.0, autonomy.CELL_CLIFF)
        grid.set_cell(20.0, 20.0, autonomy.CELL_CLEAR)
        self.assertEqual(grid.get_cell(20.0, 20.0), autonomy.CELL_CLIFF)
        mapped, cliffs = grid.counts()
        self.assertEqual(mapped, 2)
        self.assertEqual(cliffs, 1)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class AutonomyMujoco(unittest.TestCase):
    def test_standalone_dock_holds_range_during_a_lift_move(self) -> None:
        sim = autonomy.make_dock_sim(start_docked=True)
        self.assertTrue(sim.charger_contact)
        sample = sim._range_sample
        self.assertIsNotNone(sample)
        sim.command_lift_height(45)
        sim.step(150)
        self.assertTrue(sim.state.prox_is_lift_in_fov)
        self.assertIs(sim._range_sample, sample)
        sim.step(55)
        self.assertIsNot(sim._range_sample, sample)

    def test_observe_markers_accuracy_and_occlusion(self) -> None:
        sim = autonomy.make_dock_sim(start_xy_mm=(0.0, 0.0), heading_deg=180.0, head_deg=4.0, lift_mm=32.0)
        obs = autonomy.observe_markers(sim)
        charger = obs["charger"]
        self.assertIsNotNone(charger)
        assert charger is not None
        self.assertLess(abs(charger["lip_x"] - (-120.0)), 4.0)
        self.assertLess(abs(charger["lip_y"]), 4.0)
        occluded = autonomy.make_dock_sim(start_xy_mm=(185.0, 0.0), heading_deg=180.0, head_deg=4.0, lift_mm=32.0)
        obs_occ = autonomy.observe_markers(occluded)
        self.assertIsNone(obs_occ["charger"])

    def test_explore_maps_cube_and_charger(self) -> None:
        sim = autonomy.make_dock_sim(start_xy_mm=(0.0, 0.0), heading_deg=0.0, head_deg=4.0, lift_mm=32.0)
        ctrl = autonomy.DockingExplorer(sim)
        res = ctrl.explore(6.0)
        self.assertEqual(res["result"], "completed")
        self.assertGreaterEqual(res["cells_mapped"], 50)
        self.assertTrue(res["cube_seen"])
        self.assertTrue(res["charger_seen"])

    def test_undock_and_dock_multiple_poses(self) -> None:
        for label, opts in (
            ("docked", {"start_docked": True}),
            ("origin", {"start_xy_mm": (0.0, 0.0), "heading_deg": 0.0}),
            ("diagonal", {"start_xy_mm": (120.0, 80.0), "heading_deg": 45.0}),
        ):
            sim = autonomy.make_dock_sim(**opts)
            ctrl = autonomy.DockingExplorer(sim)
            if label == "docked":
                self.assertEqual(ctrl.undock(), "completed")
                self.assertFalse(autonomy.sync_dock_contact(sim)["on_charger"])
            dock_res = ctrl.dock(55.0)
            self.assertEqual(dock_res["result"], "completed", f"{label}: {dock_res}")
            self.assertTrue(autonomy.sync_dock_contact(sim)["on_charger"], label)
            self.assertAlmostEqual(sim.battery_volts, 4.2, places=2)

    def test_the_lift_holds_still_while_the_robot_turns_onto_the_dock_and_backs_on(self) -> None:
        """The lift moves only while the robot stands. Raised at the start of the turn onto the dock's axis, with
        the backup starting as the turn ended, the dock from a docked start failed for 7 of 8 noise seeds once
        the lift had the firmware's integral term (test_lift_integral).

        Each visit to a stage is measured on its own: across retries the lift is in position at 45 mm anywhere
        within its tolerance. Within one visit the lift moved at most 0.17 mm in the turn and 0.52 mm in a backup,
        as the climb changes the load on it (the three docks of test_undock_and_dock_multiple_poses, and the
        browser's alike); raised there it moves 13 mm."""
        sim = autonomy.make_dock_sim(start_docked=True)
        ctrl = autonomy.DockingExplorer(sim)
        self.assertEqual(ctrl.undock(), "completed")
        visits = []
        step = ctrl._step_dock_state

        def watch(ctx, dock_st):
            stage = ctx.stage if ctx.stage in ("align_back", "backup") else None
            if stage is not None:
                if not visits or visits[-1][0] != stage or visits[-1][2]:
                    visits.append([stage, [], False])
                visits[-1][1].append(sim.lift_height_mm())
            elif visits:
                visits[-1][2] = True
            return step(ctx, dock_st)

        ctrl._step_dock_state = watch
        self.assertEqual(ctrl.dock(55.0)["result"], "completed")
        self.assertEqual({stage for stage, _, _ in visits}, {"align_back", "backup"})
        for stage, seen, _ in visits:
            self.assertGreater(len(seen), 20, stage)
            self.assertLess(max(seen) - min(seen), 1.0, "the lift moved %.2f mm during %s" % (max(seen) - min(seen), stage))
