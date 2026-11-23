"""The lift's integral term: the firmware's angleErrorSum_ on top of the model's lift servo.

The firmware's lift power is Kp x error + Ki x error sum (+ a D term), every 5 ms control tick
(LIFTC:832-892). The model's lift was a position servo alone, a spring toward its target, and a
spring stops short under a load by the load's torque over its stiffness: carrying the light cube it
stopped at 89.8 mm of 92, outside lift_angle_tol_deg, and set_lift_height timed out. The servo stands
for Kp x error, so the sum moves the servo's target by Ki / Kp x the sum (mjcf.lift_integral_offset_rad).
The rules of the sum and a steady load are checked here; the lift carrying the cube in
test_cube_pickup.Pickup.
"""

from __future__ import annotations

import math
import unittest
import xml.etree.ElementTree as ET

import _common
from simulacrum import mjcf, params


def _floats(text):
    return [float(x) for x in text.split()]


class ControlRange(unittest.TestCase):
    def setUp(self) -> None:
        self.v = params.values(params.load())
        self.d = mjcf.derive(self.v)
        self.root = ET.fromstring(mjcf.build_mjcf(self.v))

    def test_the_offset_is_ki_over_kp_of_the_sum(self) -> None:
        kp, ki, _ = self.v["lift_power_gains"]
        self.assertAlmostEqual(mjcf.lift_integral_offset_rad(self.v, 3.0), ki / kp * 3.0, places=12)
        self.assertAlmostEqual(mjcf.lift_integral_offset_rad(self.v, -3.0), -ki / kp * 3.0, places=12)

    def test_the_servo_may_aim_past_either_end_by_the_largest_offset_and_the_joint_may_not(self) -> None:
        reach = mjcf.lift_integral_offset_rad(self.v, self.v["lift_error_sum_max"])
        self.assertAlmostEqual(reach, 0.1 / 3.0 * 5.0, places=9)
        servo = [a for a in self.root.iter("position") if a.get("name") == "lift"]
        joint = [j for j in self.root.iter("joint") if j.get("name") == "lift_joint"]
        self.assertEqual((len(servo), len(joint)), (1, 1))
        low, high = self.d["lift_angle_min"], self.d["lift_angle_max"]
        lo, hi = _floats(servo[0].get("ctrlrange"))
        self.assertAlmostEqual(lo, low - reach, places=5)
        self.assertAlmostEqual(hi, high + reach, places=5)
        jlo, jhi = _floats(joint[0].get("range"))
        self.assertAlmostEqual(jlo, low, places=5)
        self.assertAlmostEqual(jhi, high, places=5)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class ErrorSum(unittest.TestCase):
    def setUp(self) -> None:
        from simulacrum import robot
        self.step = robot.lift_error_sum_step
        self.v = params.values(params.load())
        self.tol = math.radians(self.v["lift_angle_tol_deg"])
        self.cap = self.v["lift_error_sum_max"]

    def test_off_target_or_while_the_target_moves_the_error_goes_on_the_sum(self) -> None:
        self.assertAlmostEqual(self.step(self.v, 0.3, 2.0 * self.tol, True), 0.3 + 2.0 * self.tol, places=12)
        self.assertAlmostEqual(self.step(self.v, 0.3, -2.0 * self.tol, True), 0.3 - 2.0 * self.tol, places=12)
        self.assertAlmostEqual(self.step(self.v, 0.3, 0.5 * self.tol, False), 0.3 + 0.5 * self.tol, places=12)

    def test_the_sum_is_clipped_both_ways(self) -> None:
        self.assertEqual(self.step(self.v, self.cap - 0.01, 1.0, True), self.cap)
        self.assertEqual(self.step(self.v, -self.cap + 0.01, -1.0, True), -self.cap)
        self.assertEqual(self.step(self.v, 10.0 * self.cap, 0.0, False), self.cap)

    def test_holding_its_target_with_little_power_the_lift_leaves_the_sum_alone(self) -> None:
        kp, ki, _ = self.v["lift_power_gains"]
        small = 0.5 * self.v["lift_power_in_position_max"] / ki
        self.assertEqual(self.step(self.v, small, 0.0, True), small)
        self.assertEqual(self.step(self.v, -small, 0.0, True), -small)
        self.assertLess(kp * 0.5 * self.tol + ki * small, 1.0, "fixture must stay a modest power")

    def test_holding_its_target_with_more_power_the_lift_bleeds_the_sum_toward_zero(self) -> None:
        decay = self.v["lift_error_sum_decay"]
        large = 2.0 * self.v["lift_power_in_position_max"] / self.v["lift_power_gains"][1]
        self.assertAlmostEqual(self.step(self.v, large, 0.0, True), large - decay, places=12)
        self.assertAlmostEqual(self.step(self.v, -large, 0.0, True), -large + decay, places=12)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Retarget(unittest.TestCase):
    def setUp(self) -> None:
        from simulacrum import robot
        self.v = params.values(params.load())
        self.sim = robot.VectorSim(values=self.v, lift_mm=self.v["lift_height_max_mm"])
        self.sim.advance(0.5)
        self.assertTrue(self.sim._lift_in_pos, "the lift must start in position")

    def test_a_new_target_reaching_an_in_position_lift_clears_the_sum(self) -> None:
        self.sim._lift_error_sum = 2.0
        self.assertTrue(self.sim.command_lift_height(self.v["lift_height_min_mm"]))
        self.assertEqual(self.sim._lift_error_sum, 0.0)
        self.assertFalse(self.sim._lift_in_pos)

    def test_a_new_target_on_a_lift_still_moving_keeps_the_sum(self) -> None:
        self.assertTrue(self.sim.command_lift_height(self.v["lift_height_min_mm"]))
        self.sim._lift_error_sum = 2.0
        self.assertTrue(self.sim.command_lift_height(60.0))
        self.assertEqual(self.sim._lift_error_sum, 2.0)

    def test_the_same_target_again_changes_nothing(self) -> None:
        self.sim._lift_error_sum = 2.0
        self.assertTrue(self.sim.command_lift_height(self.v["lift_height_max_mm"]))
        self.assertEqual(self.sim._lift_error_sum, 2.0)
        self.assertTrue(self.sim._lift_in_pos)

    def test_stopping_an_in_position_lift_clears_the_sum(self) -> None:
        self.sim._lift_error_sum = 2.0
        self.assertTrue(self.sim.stop_all_motors())
        self.assertEqual(self.sim._lift_error_sum, 0.0)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class SteadyLoad(unittest.TestCase):
    """A steady torque pulling the lift down, about what the light cube costs (holding_torque.py: 38 mN*m)."""

    LOAD_NM = 0.038

    def _raise(self, v):
        from simulacrum import robot
        sim = robot.VectorSim(values=v)
        sim.data.qfrc_applied[sim._dof["lift"]] = -self.LOAD_NM
        result = sim.set_lift_height(v["lift_height_max_mm"])
        short = mjcf.lift_angle_of_height(v, v["lift_height_max_mm"]) - float(sim.data.qpos[sim._q["lift"]])
        return result, short

    def test_under_load_the_lift_reaches_its_target_and_the_action_completes(self) -> None:
        v = params.values(params.load())
        result, short = self._raise(v)
        self.assertEqual(result.result, "completed")
        self.assertLess(short, math.radians(v["lift_angle_tol_deg"]))

    def test_with_no_integral_gain_the_loaded_lift_stops_short_and_times_out(self) -> None:
        """The control: the model before the term, the servo alone."""
        base = params.values(params.load())
        kp, _, kd = base["lift_power_gains"]
        v = dict(base, lift_power_gains=[kp, 0.0, kd])
        result, short = self._raise(v)
        self.assertEqual(result.result, "timeout")
        self.assertGreater(short, math.radians(v["lift_angle_tol_deg"]))


if __name__ == "__main__":
    unittest.main()
