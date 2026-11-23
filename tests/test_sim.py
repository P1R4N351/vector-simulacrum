"""The simulated robot behaves as the table and the SDK say. Needs MuJoCo..

Tolerances, under nominal parameters:
  straight drive   at 20 mm/s: 3 mm along the path. Faster: never short by more than
                   3 mm, and never long by more than the treads can coast (speed times
                   the drive's time constant), because the firmware does not brake.
                   2 mm sideways, 1 degree of heading
  turn in place    2 degrees (the manual's own tolerance, turn_tolerance_deg)
  cliff range      0.3 mm on the table, 3 mm over the edge
  forward range    0.3 mm against the geometry, with sensor noise switched off
"""

from __future__ import annotations

import math
import unittest

import _common
from simulacrum import mjcf, params

if _common.HAVE_MUJOCO:
    import mujoco
    from simulacrum import robot
    from simulacrum.robot import CONTROL_DEFAULT, CONTROL_OVERRIDE, VectorSim

DRIVE_ALONG_TOL_MM = 3.0
DRIVE_SIDE_TOL_MM = 2.0
DRIVE_HEADING_TOL_DEG = 1.0
CLIFF_TABLE_TOL_MM = 0.3
CLIFF_EDGE_TOL_MM = 3.0
TOF_TOL_MM = 0.3
EDGE_START_MM = 150.0
QUIET = {"tof_range_std_fraction": 0.0, "tof_weak_distance_sd_mm": 0.0, "gyro_noise_rms_dps": 0.0, "accel_noise_rms_mg": 0.0}
WALL = (150.0, 0.0, 50.0, 10.0, 200.0, 100.0)  # centre and size in mm: face at x = 145


def _coast_mm(v: dict, speed_mmps: float) -> float:
    """The furthest a tread can run on after its power is cut: speed times the drive's time constant."""
    radius = v["wheel_front_radius_mm"] * 0.001
    inertia = (v["wheel_armature_gcm2"] * 1e-7 * (1.0 + mjcf.derive(v)["rear_gear"] ** 2)
               + 0.5 * v["total_mass_g"] * 0.001 * radius * radius)
    return abs(speed_mmps) * inertia / mjcf.tread_drive(v)["damping_Nms_per_rad"]


def _quiet_sim(**kwargs: object) -> "VectorSim":
    """A nominal robot with sensor noise switched off, for checks against geometry."""
    return VectorSim(values=params.values(params.load(), QUIET), **kwargs)


def _edge_sim(level: str, **overrides: float) -> "VectorSim":
    v = params.values(params.load(), overrides)
    return VectorSim(values=v, control_level=level, start_xy_mm=(0.5 * v["table_length_mm"] - EDGE_START_MM, 0.0))


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class ModelLoads(unittest.TestCase):
    def test_model_compiles_with_the_expected_shape(self) -> None:
        sim = VectorSim()
        self.assertEqual(sim.model.nu, 4)
        self.assertEqual(sim.model.nsensor, 6)
        self.assertEqual(sim.model.nq, 7 + 4 + 3)
        self.assertEqual(sim.model.ncam, 1)

    def test_fork_side_plates_contact_a_probe_and_the_opening_passes_it(self) -> None:
        v = params.values(params.load())
        # Neutral shoulder: wrist at (25, 0, 45) mm. The probe is above the
        # crossbar, where the original crossbar-only collider missed both sides.
        for y, plate in [(22, "fork_side_left"), (-22, "fork_side_right"), (0, None)]:
            model = mujoco.MjModel.from_xml_string(mjcf.build_mjcf(v, [(27, y, 35, 1, 1, 1)]))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            contacts = [{model.geom(int(g)).name for g in pair} for pair in data.contact.geom]
            probe = [pair for pair in contacts if "obstacle_0" in pair]
            self.assertEqual(bool(probe), plate is not None)
            if plate:
                self.assertTrue(all(plate in pair for pair in probe))

    def test_compiled_mass_and_centre_of_mass_match_the_table(self) -> None:
        v = params.values(params.load())
        model = mujoco.MjModel.from_xml_string(mjcf.build_mjcf(v))
        data = mujoco.MjData(model)
        lift = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift_joint")
        wrist = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "wrist_joint")
        low = mjcf.derive(v)["lift_angle_min"]
        data.qpos[model.jnt_qposadr[lift]], data.qpos[model.jnt_qposadr[wrist]] = low, -low
        mujoco.mj_forward(model, data)
        chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self.assertAlmostEqual(float(model.body_subtreemass[chassis]) * 1000.0, v["total_mass_g"], places=6)
        com = [float(c) * 1000.0 for c in data.subtree_com[chassis]]
        self.assertAlmostEqual(com[0], v["com_x_mm"], delta=0.05)
        self.assertAlmostEqual(com[1], 0.0, delta=0.05)
        self.assertAlmostEqual(com[2], v["com_z_mm"], delta=0.05)

    def test_robot_at_rest_stays_put_and_reports_gravity(self) -> None:
        sim = VectorSim()
        state = sim.advance(2.0)
        self.assertLess(math.hypot(state.x_mm, state.y_mm), 0.05)
        self.assertLess(abs(state.z_mm), 0.2)
        self.assertLess(abs(math.degrees(state.pose_angle_rad)), 0.05)
        z_offset = 0.001 * sim.v["gravity_mmps2"] * sim.v["accel_bias_mg"][2]
        self.assertAlmostEqual(state.accel_mmps2[2], sim.v["gravity_mmps2"] + z_offset, delta=100.0)
        self.assertFalse(state.cliff_detected or state.picked_up or state.falling or state.wheels_moving)
        self.assertTrue(state.has_control)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Vector2(unittest.TestCase):
    def test_the_2_0_set_and_drawn_2_0_robots_build_settle_and_spin(self) -> None:
        doc = params.load(model="vector_2")
        for values in (params.values(doc), params.sample(doc, 3), params.sample(doc, 4)):
            sim = VectorSim(values=values)
            sim.advance(0.5)
            z0 = sim.pose_mm()[2]
            sim.set_wheel_motors(-100.0, 100.0)
            # Added up tick by tick: the heading wraps at 180 degrees, so a near full turn would read as a small one.
            turned, last = 0.0, sim.state.pose_angle_rad
            for _ in range(int(round(2.0 / sim.dt))):
                now = sim.step(1).pose_angle_rad
                turned += math.remainder(now - last, 2.0 * math.pi)
                last = now
            self.assertGreater(abs(math.degrees(turned)), 20.0, "it turned")
            self.assertLess(abs(sim.pose_mm()[2] - z0), 3.0, "it stayed on its treads")


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Drive(unittest.TestCase):
    def _check_straight(self, dist_mm: float, speed_mmps: float) -> None:
        sim = VectorSim()
        result = sim.drive_straight(dist_mm, speed_mmps)
        x, y, _ = sim.pose_mm()
        overrun = math.copysign(1.0, dist_mm) * (x - dist_mm)
        self.assertEqual(result.result, robot.RESULT_COMPLETED)
        self.assertGreater(overrun, -DRIVE_ALONG_TOL_MM, "a drive must not stop short")
        self.assertLess(overrun, _coast_mm(sim.v, speed_mmps) + DRIVE_ALONG_TOL_MM, "nor run on further than it can coast")
        self.assertLess(abs(y), DRIVE_SIDE_TOL_MM)
        self.assertLess(abs(math.degrees(sim.heading_total_rad())), DRIVE_HEADING_TOL_DEG)
        self.assertAlmostEqual(result.moved_mm, abs(x), delta=0.5)

    def test_slow_straight_drive_lands_on_the_distance(self) -> None:
        sim = VectorSim()
        result = sim.drive_straight(50.0, sim.v["directive_forward_speed_min_mmps"])
        self.assertEqual(result.result, robot.RESULT_COMPLETED)
        self.assertAlmostEqual(sim.pose_mm()[0], 50.0, delta=DRIVE_ALONG_TOL_MM)

    def test_straight_drive_forward_overruns_by_no_more_than_the_coast(self) -> None:
        self._check_straight(200.0, 100.0)

    def test_straight_drive_backward_overruns_by_no_more_than_the_coast(self) -> None:
        self._check_straight(-100.0, 60.0)

    def test_straight_drive_at_the_fastest_directive_speed(self) -> None:
        v = params.values(params.load())
        self._check_straight(250.0, v["directive_forward_speed_max_mmps"])

    def test_cutting_the_command_coasts_and_never_powers_backwards(self) -> None:
        sim = VectorSim()
        sim.set_wheel_motors(100.0, 100.0)
        sim.advance(1.5)
        x0 = sim.pose_mm()[0]
        sim.set_wheel_motors(0.0, 0.0)
        lowest = 0.0
        for _ in range(300):
            sim.step(1)
            lowest = min(lowest, *sim.sim_truth()["motor_power"])
        coast = sim.pose_mm()[0] - x0
        self.assertEqual(lowest, 0.0, "the firmware never applies power against the direction of travel")
        self.assertGreater(coast, 5.0)
        self.assertLess(coast, _coast_mm(sim.v, 100.0) + 100.0 * sim.v["command_delay_ms"] / 1000.0 + 1.0)

    def test_encoder_counts_follow_the_distance_driven(self) -> None:
        sim = VectorSim()
        sim.drive_straight(150.0, 60.0)
        left, right = sim.sim_truth()["encoder_counts"]
        expected = sim.pose_mm()[0] / sim.v["tread_mm_per_encoder_count"]
        self.assertAlmostEqual(left, expected, delta=0.01 * expected)
        self.assertAlmostEqual(right, expected, delta=0.01 * expected)

    def test_straight_drive_follows_the_start_heading(self) -> None:
        sim = VectorSim(heading_deg=90.0)
        sim.drive_straight(150.0, 20.0)
        x, y, _ = sim.pose_mm()
        self.assertAlmostEqual(y, 150.0, delta=DRIVE_ALONG_TOL_MM)
        self.assertLess(abs(x), DRIVE_SIDE_TOL_MM)

    def _check_turn(self, angle_deg: float, speed_dps: float) -> None:
        sim = VectorSim()
        result = sim.turn_in_place(angle_deg, speed_dps)
        self.assertEqual(result.result, robot.RESULT_COMPLETED)
        self.assertAlmostEqual(result.turned_deg, angle_deg, delta=sim.v["turn_tolerance_deg"])
        self.assertAlmostEqual(math.degrees(sim.heading_total_rad()), angle_deg, delta=sim.v["turn_tolerance_deg"])
        x, y, _ = sim.pose_mm()
        self.assertLess(math.hypot(x, y), 2.0 * sim.v["wheel_base_mm"], "a turn in place must not travel")

    def test_turn_left_quarter(self) -> None:
        self._check_turn(90.0, 60.0)

    def test_turn_right_half(self) -> None:
        self._check_turn(-180.0, 120.0)

    def test_turn_small(self) -> None:
        self._check_turn(30.0, 60.0)

    def test_wheel_speed_command_is_tracked_in_mm_per_s(self) -> None:
        sim = VectorSim(values=params.values(params.load(), {"table_length_mm": 1500.0}), start_xy_mm=(-600.0, 0.0))
        self.assertTrue(sim.set_wheel_motors(100.0, 100.0))
        sim.advance(4.0)
        x0 = sim.pose_mm()[0]
        state = sim.advance(1.0)
        self.assertAlmostEqual(sim.pose_mm()[0] - x0, 100.0, delta=2.0)
        self.assertAlmostEqual(state.left_wheel_speed_mmps, 100.0, delta=2.0)
        self.assertAlmostEqual(state.right_wheel_speed_mmps, 100.0, delta=2.0)
        self.assertEqual(state.left_wheel_speed_mmps, int(state.left_wheel_speed_mmps), "reported in whole mm/s")
        self.assertTrue(state.wheels_moving)

    def test_direct_drive_is_limited_by_the_motor_not_the_firmware_constant(self) -> None:
        """set_wheel_motors is not clamped to 220 (the firmware clamps in path following only); the motor's
        ceiling at full duty sets the speed, and a lower battery lowers it (REC2 run F reached 243-247)."""
        speeds = {}
        for volts in (4.03, 3.7):
            v = params.values(params.load(), {"table_length_mm": 1500.0, "battery_rest_voltage_v": volts})
            sim = VectorSim(values=v, start_xy_mm=(-650.0, 0.0))
            sim.set_wheel_motors(500.0, 500.0)
            sim.advance(2.0)
            x0 = sim.pose_mm()[0]
            sim.advance(1.0)
            speeds[volts] = sim.pose_mm()[0] - x0
        self.assertGreater(speeds[4.03], sim.v["wheel_speed_max_mmps"] + 15.0)
        self.assertLess(speeds[3.7], speeds[4.03] - 10.0)

    def test_stop_all_motors_stops_the_robot(self) -> None:
        sim = VectorSim()
        sim.set_wheel_motors(80.0, 80.0)
        sim.advance(0.5)
        self.assertTrue(sim.stop_all_motors())
        sim.advance(1.5)
        x0 = sim.pose_mm()[0]
        state = sim.advance(0.5)
        self.assertLess(abs(sim.pose_mm()[0] - x0), 0.1)
        self.assertFalse(state.wheels_moving)

    def test_command_takes_effect_after_the_command_delay(self) -> None:
        sim = VectorSim()
        delay_ticks = int(round(sim.v["command_delay_ms"] / sim.v["control_period_ms"]))
        self.assertGreaterEqual(delay_ticks, 2)
        sim.set_wheel_motors(100.0, 100.0)
        sim.step(delay_ticks)
        self.assertLess(max(abs(w) for w in sim.sim_truth()["wheel_speeds_mmps"]), 0.05)
        sim.step(4)
        self.assertGreater(sim.sim_truth()["wheel_speeds_mmps"][0], 1.0)

    def test_reported_wheel_speed_takes_its_sign_from_the_command(self) -> None:
        def reverse_then_forward(sim: "VectorSim") -> tuple:
            sim.set_wheel_motors(-150.0, -150.0)
            sim.advance(1.0)
            sim.set_wheel_motors(30.0, 30.0)
            state = sim.advance(0.15)
            return sim.sim_truth()["wheel_speeds_mmps"][0], state.left_wheel_speed_mmps

        true_speed, reported = reverse_then_forward(VectorSim())
        self.assertLess(true_speed, -10.0, "the tread is still coasting backwards")
        self.assertGreater(reported, 10.0, "one encoder channel cannot see that; the sign is the command's")
        sensed = VectorSim(values=params.values(params.load(), {"encoder_tread_direction_sensed": True}))
        self.assertLess(reverse_then_forward(sensed)[1], -10.0)

    def test_state_messages_come_at_the_state_rate(self) -> None:
        sim = VectorSim()
        before = len(sim.states)
        sim.advance(3.0)
        self.assertEqual(len(sim.states) - before, int(round(3000.0 / sim.v["state_period_ms"])))
        gap = sim.states[-1].time_s - sim.states[-2].time_s
        self.assertAlmostEqual(gap * 1000.0, sim.v["state_period_ms"], places=6)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Servos(unittest.TestCase):
    def test_head_reaches_its_limits_and_is_clamped(self) -> None:
        sim = VectorSim()
        tol = sim.v["head_angle_tol_deg"]
        self.assertEqual(sim.set_head_angle(45.0).result, robot.RESULT_COMPLETED)
        sim.advance(0.5)
        self.assertAlmostEqual(math.degrees(sim.head_angle_rad()), 45.0, delta=tol)
        self.assertEqual(sim.set_head_angle(-90.0).result, robot.RESULT_COMPLETED)
        sim.advance(0.5)
        self.assertAlmostEqual(math.degrees(sim.head_angle_rad()), sim.v["head_angle_min_deg"], delta=tol)
        self.assertAlmostEqual(math.degrees(sim.state.head_angle_rad), sim.v["head_angle_min_deg"], delta=tol)
        counts = sim.state.head_angle_rad / sim.v["head_rad_per_encoder_count"]
        self.assertAlmostEqual(counts, round(counts), places=6, msg="head angle is reported in whole encoder counts")

    def test_lift_encoder_counts_begin_at_calibrated_low_angle(self) -> None:
        sim = VectorSim()
        low, step = sim.d["lift_angle_min"], sim.v["lift_rad_per_encoder_count"]
        for offset, counts in ((0, 0), (0.45, 0), (6.55, 7), (-1.1, -1)):
            sim.data.qpos[sim._q["lift"]] = low + offset * step
            expected = sim.v["lift_base_position_mm"][2] + sim.v["lift_arm_length_mm"] * math.sin(low + counts * step)
            self.assertAlmostEqual(sim.reported_now()["lift_mm"], expected)

    def test_lift_status_clears_on_motion_and_latches_after_settling(self) -> None:
        sim = _quiet_sim()
        self.assertTrue(sim.state.lift_in_pos)
        sim.command_lift_height(62, duration_s=0.7)
        self.assertFalse(sim.step(sim._state_ticks).lift_in_pos)
        self.assertTrue(sim.step(round(1.5 / sim.dt)).lift_in_pos)
        sim.command_lift_height(62)
        self.assertTrue(sim.step(sim._state_ticks).lift_in_pos)
        sim.stop_all_motors()
        self.assertFalse(sim.step(sim._state_ticks).lift_in_pos)
        self.assertTrue(sim.step(round(0.5 / sim.dt)).lift_in_pos)
        sim.command_lift_height(32)
        self.assertFalse(sim.step(sim._state_ticks).lift_in_pos)

    def test_lift_duration_applies_to_one_action(self) -> None:
        sim = VectorSim()
        timed = sim.set_lift_height(92.0, duration_s=1.4)
        self.assertEqual(timed.result, robot.RESULT_COMPLETED)
        self.assertGreater(timed.ticks * sim.dt, 1.2)
        sim.advance(0.5)
        default = sim.set_lift_height(32.0)
        self.assertEqual(default.result, robot.RESULT_COMPLETED)
        self.assertLess(default.ticks * sim.dt, 1.0)

    def test_lift_reaches_its_limits_and_is_clamped(self) -> None:
        sim = VectorSim()
        self.assertEqual(sim.set_lift_height(92.0).result, robot.RESULT_COMPLETED)
        sim.advance(0.5)
        self.assertAlmostEqual(sim.lift_height_mm(), 92.0, delta=1.0)
        self.assertFalse(sim.state.prox_is_lift_in_fov)
        self.assertEqual(sim.set_lift_height(5.0).result, robot.RESULT_COMPLETED)
        sim.advance(0.5)
        self.assertAlmostEqual(sim.lift_height_mm(), sim.v["lift_height_min_mm"], delta=1.0)
        self.assertFalse(sim.state.prox_is_lift_in_fov, "fully down, the lift is below the beam")

    def test_reported_lift_height_is_the_height_of_the_wrist(self) -> None:
        sim = VectorSim()
        sim.set_lift_height(70.0)
        sim.advance(0.5)
        fork = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "lift_fork")
        self.assertAlmostEqual(float(sim.data.xpos[fork][2]) * 1000.0, sim.lift_height_mm(), delta=0.5)
        self.assertAlmostEqual(sim.lift_height_mm(), 70.0, delta=1.0)

    def test_head_and_lift_rates_change_nothing_while_neither_is_commanded(self) -> None:
        def drive(overrides: dict) -> list:
            sim = VectorSim(values=params.values(params.load(), overrides), head_deg=20.0, lift_mm=60.0, noise_seed=3)
            sim.set_wheel_motors(80.0, -40.0)
            sim.advance(2.0)
            return [(s.x_mm, s.y_mm, s.left_wheel_speed_mmps, s.gyro_radps, s.head_angle_rad) for s in sim.states]

        slow = drive({"head_speed_max_radps": 1.3, "lift_speed_max_radps": 2.0})
        fast = drive({"head_speed_max_radps": 4.1, "lift_speed_max_radps": 6.0})
        self.assertGreater(len(slow), 30)
        self.assertEqual(slow, fast)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class RangeSensors(unittest.TestCase):
    def test_body_range_clock_holds_sample_across_lift_and_charger_changes(self) -> None:
        sim = VectorSim(values=params.values(params.load(model="vector_2")), cube="free",
                        body_range_clock=True, charger_contact=True)
        initial = sim.state.prox_distance_mm
        self.assertTrue(sim.command_lift_height(45))
        sim.step(150)
        self.assertTrue(sim.state.prox_is_lift_in_fov)
        self.assertEqual(sim.state.prox_distance_mm, initial)
        changes = []
        publications = []
        sample = published = sim._range_sample
        for _ in range(480):
            state = sim.state
            sim.step()
            if sample is not sim._range_sample:
                changes.append(sim.tick)
                sample = sim._range_sample
            if sim.state is not state and published is not sample:
                publications.append(sim.tick)
                published = sample
        self.assertEqual(changes, [205, 410, 615])
        self.assertEqual(publications, [216, 420, 624])
        self.assertEqual(sim.state.prox_distance_mm, 30)
        sim.charger_contact = False
        sim.step(2)
        self.assertIsNot(sim._range_sample, sample)

    def test_cliff_sensors_read_their_height_on_the_table(self) -> None:
        sim = _quiet_sim()
        sim.advance(0.2)
        for reading in sim.cliff_ranges_mm():
            self.assertAlmostEqual(reading, sim.v["cliff_sensor_height_mm"], delta=CLIFF_TABLE_TOL_MM)
        self.assertEqual(sim.sim_truth()["cliff_flags"], [False, False, False, False])

    def test_front_cliff_sensors_over_the_edge_read_the_floor(self) -> None:
        v = params.values(params.load())
        start = 0.5 * v["table_length_mm"] - v["cliff_front_x_mm"] + 3.0
        sim = VectorSim(values=v, start_xy_mm=(start, 0.0))
        sim.step(2)
        front_left, front_right, back_left, back_right = sim.cliff_ranges_mm()
        drop = v["table_height_mm"] + v["cliff_sensor_height_mm"]
        self.assertAlmostEqual(front_left, drop, delta=CLIFF_EDGE_TOL_MM)
        self.assertAlmostEqual(front_right, drop, delta=CLIFF_EDGE_TOL_MM)
        self.assertAlmostEqual(back_left, v["cliff_sensor_height_mm"], delta=CLIFF_EDGE_TOL_MM)
        self.assertAlmostEqual(back_right, v["cliff_sensor_height_mm"], delta=CLIFF_EDGE_TOL_MM)
        self.assertEqual(sim.sim_truth()["cliff_flags"], [True, True, False, False])
        sim.step(sim._state_ticks)
        self.assertTrue(sim.state.cliff_detected, "one status bit covers all four sensors")

    def test_forward_range_reads_a_wall_that_fills_its_cone(self) -> None:
        sim = _quiet_sim(obstacles=[WALL])
        state = sim.advance(0.2)
        v = sim.v
        face_x = WALL[0] - 0.5 * WALL[3]
        on_axis = (face_x - v["tof_position_mm"][0]) / math.cos(math.radians(v["tof_tilt_up_deg"]))
        hits = sim.sim_truth()["tof_hits_mm"]
        self.assertEqual(len(hits), 19)
        self.assertTrue(all(h > 0.0 for h in hits), "every ray of the fan lands on the wall")
        self.assertAlmostEqual(hits[0], on_axis, delta=TOF_TOL_MM)
        self.assertTrue(min(hits) <= state.prox_distance_mm <= max(hits) + 1.0)
        self.assertAlmostEqual(state.prox_distance_mm, on_axis, delta=4.0)
        self.assertEqual(state.prox_distance_mm, int(state.prox_distance_mm), "reported in whole mm")
        self.assertGreater(state.prox_signal_quality, v["tof_quality_min"])
        self.assertTrue(state.prox_found_object)
        self.assertFalse(state.prox_unobstructed)

    def test_forward_range_scatters_by_the_table_fraction(self) -> None:
        quiet = _quiet_sim(obstacles=[WALL]).advance(0.2).prox_distance_mm
        sim = VectorSim(obstacles=[WALL])
        readings = [sim.step(sim._state_ticks).prox_distance_mm for _ in range(300)]
        mean = sum(readings) / len(readings)
        spread = math.sqrt(sum((r - mean) ** 2 for r in readings) / len(readings))
        self.assertAlmostEqual(mean, quiet, delta=1.5)
        self.assertAlmostEqual(spread / mean, sim.v["tof_range_std_fraction"], delta=0.008)

    def test_the_lift_in_the_beam_reads_the_minimum_and_hides_what_is_behind_it(self) -> None:
        v = params.values(params.load(), QUIET)
        blocked = VectorSim(values=v, obstacles=[WALL], lift_mm=45.0).advance(0.2)
        self.assertTrue(blocked.prox_is_lift_in_fov)
        self.assertEqual(blocked.prox_distance_mm, v["tof_report_min_mm"])
        self.assertEqual(blocked.prox_signal_quality, v["tof_lift_quality"])
        self.assertFalse(blocked.prox_found_object, "the lift is not an object found")
        for height in (v["lift_height_min_mm"], 80.0):
            clear = VectorSim(values=v, obstacles=[WALL], lift_mm=height).advance(0.2)
            self.assertFalse(clear.prox_is_lift_in_fov, f"lift at {height} mm")
            self.assertTrue(clear.prox_found_object, f"lift at {height} mm")

    def test_the_lift_counts_as_in_view_for_a_moment_after_it_leaves_the_beam(self) -> None:
        sim = _quiet_sim(lift_mm=45.0)
        self.assertTrue(sim.advance(0.2).prox_is_lift_in_fov)
        self.assertEqual(sim.set_lift_height(92.0).result, robot.RESULT_COMPLETED)
        sim.advance(0.3)
        states = list(sim.states)
        last_in = max(i for i, s in enumerate(states) if s.lift_height_mm <= sim.v["tof_lift_occluding_max_mm"])
        self.assertTrue(states[last_in + 1].prox_is_lift_in_fov, "still in view in the first message after it left")
        self.assertFalse(states[last_in + 2].prox_is_lift_in_fov, "and out by the second")

    def test_gyro_and_accelerometer_carry_the_table_noise_and_bias(self) -> None:
        biased = params.values(params.load(), {"gyro_bias_dps": [0.3, 0.0, 0.0], "accel_bias_mg": [0.0, 30.0, 0.0]})
        sim = VectorSim(values=biased)
        states = [sim.step(sim._state_ticks) for _ in range(300)]
        gyro_x = [math.degrees(s.gyro_radps[0]) for s in states]
        gyro_z = [math.degrees(s.gyro_radps[2]) for s in states]
        accel_y = [s.accel_mmps2[1] for s in states]
        mean = lambda xs: sum(xs) / len(xs)
        spread = lambda xs: math.sqrt(sum((x - mean(xs)) ** 2 for x in xs) / len(xs))
        self.assertAlmostEqual(mean(gyro_x), 0.3, delta=0.03)
        self.assertAlmostEqual(spread(gyro_z), sim.v["gyro_noise_rms_dps"], delta=0.02)
        self.assertAlmostEqual(mean(accel_y), 30.0 * 9.81, delta=6.0)
        self.assertAlmostEqual(spread(accel_y), sim.v["accel_noise_rms_mg"] * 9.81, delta=5.0)
        quiet = _quiet_sim().advance(1.0)
        self.assertLess(abs(math.degrees(quiet.gyro_radps[2])), 0.01)

    def test_accelerometer_at_rest_reads_gravity_in_the_heads_pitched_frame_plus_the_offsets(self) -> None:
        v = params.values(params.load())
        g, mg = v["gravity_mmps2"], 0.001 * v["gravity_mmps2"]
        for head in (-20.0, 0.0, 22.4, 44.0):
            state = _quiet_sim(head_deg=head).advance(0.5)
            angle = state.head_angle_rad + math.radians(v["imu_pitch_offset_deg"])
            want = (g * math.sin(angle) + mg * v["accel_bias_mg"][0], mg * v["accel_bias_mg"][1],
                    g * math.cos(angle) + mg * v["accel_bias_mg"][2])
            for got, expected in zip(state.accel_mmps2, want):
                self.assertAlmostEqual(got, expected, delta=3.0 * mg)
        level = _quiet_sim().advance(0.5)
        self.assertLess(level.accel_mmps2[0], -50.0 * mg)

    def test_the_x_scale_multiplies_the_reading_of_gravity_before_the_offset(self) -> None:
        head = 30.0
        readings = []
        for kx in (1.0, 1.05):
            sim = VectorSim(values=params.values(params.load(), dict(QUIET, accel_scale=[kx, 1.0, 1.0])), head_deg=head)
            readings.append(sim.advance(0.5))
        v = params.values(params.load())
        g, mg = v["gravity_mmps2"], 0.001 * v["gravity_mmps2"]
        angle = readings[0].head_angle_rad + math.radians(v["imu_pitch_offset_deg"])
        self.assertAlmostEqual(readings[1].accel_mmps2[0] - readings[0].accel_mmps2[0], 0.05 * g * math.sin(angle), delta=3.0 * mg)
        self.assertAlmostEqual(readings[1].accel_mmps2[2], readings[0].accel_mmps2[2], delta=3.0 * mg)

    def _gyro_pitch_in_a_spin_deg(self, head_deg: float, offset_deg: float) -> float:
        """Pitch of the gyro's frame as a spin shows it: atan(gyro x / gyro z), mean of both turn directions,
        over 120 state periods (7.2 s): the treads' rocking leaks into gyro x, and 20 periods were too few
        for it to average out once the contact inset changed it (up - down came out at 40.0)."""
        found = []
        for speed in (100.0, -100.0):
            sim = VectorSim(values=params.values(params.load(), dict(QUIET, imu_pitch_offset_deg=offset_deg)),
                            head_deg=head_deg)
            sim.set_wheel_motors(-speed, speed)
            sim.advance(1.5)
            states = [sim.step(sim._state_ticks) for _ in range(120)]
            found.append(math.degrees(math.atan(sum(s.gyro_radps[0] for s in states)
                                                / sum(s.gyro_radps[2] for s in states))))
        return 0.5 * (found[0] + found[1])

    def test_a_spin_shows_on_gyro_x_by_the_tangent_of_the_heads_pitch(self) -> None:
        # Within one turn direction the model's gyro x carries a few deg/s that do not reverse with the turn,
        # so one direction alone is off by about 2 degrees; the mean of both is within about 1.
        up, level, down = (self._gyro_pitch_in_a_spin_deg(head, 0.0) for head in (22.4, 0.0, -15.0))
        self.assertAlmostEqual(up - down, 37.4, delta=1.0)
        self.assertAlmostEqual(up - level, 22.4, delta=1.0)
        self.assertAlmostEqual(level, 0.0, delta=1.5)
        tilted = self._gyro_pitch_in_a_spin_deg(0.0, -5.0)
        self.assertAlmostEqual(tilted - level, -5.0, delta=0.3)

    def test_forward_range_with_nothing_ahead_reports_a_weak_reading(self) -> None:
        sim = _quiet_sim()
        state = sim.advance(0.2)
        v = sim.v
        grazes = [h for h in sim.sim_truth()["tof_hits_mm"] if h >= 0.0]
        self.assertTrue(grazes and min(grazes) > 150.0, "the lowest rays reach the table top, far ahead and at a slant")
        self.assertFalse(state.prox_found_object, "the table top is not an object found")
        self.assertTrue(state.prox_unobstructed)
        self.assertEqual(state.prox_signal_quality, v["tof_quality_floor"])
        self.assertEqual(state.prox_distance_mm, v["tof_weak_distance_mm"], "the real sensor still reports a distance")

    def test_weak_readings_scatter_as_the_recordings_do(self) -> None:
        sim = VectorSim()
        readings = [sim.step(sim._state_ticks).prox_distance_mm for _ in range(400)]
        mean = sum(readings) / len(readings)
        spread = math.sqrt(sum((r - mean) ** 2 for r in readings) / len(readings))
        self.assertAlmostEqual(mean, sim.v["tof_weak_distance_mm"], delta=4.0)
        self.assertAlmostEqual(spread, sim.v["tof_weak_distance_sd_mm"], delta=4.0)
        self.assertGreaterEqual(min(readings), sim.v["tof_report_min_mm"])

    def test_forward_range_never_reports_below_its_minimum(self) -> None:
        near = (35.0, 0.0, 50.0, 10.0, 200.0, 100.0)  # face 20 mm from the sensor
        sim = _quiet_sim(obstacles=[near])
        state = sim.advance(0.05)
        self.assertLess(sim.sim_truth()["tof_hits_mm"][0], sim.v["tof_report_min_mm"])
        self.assertEqual(state.prox_distance_mm, sim.v["tof_report_min_mm"], "clamped, as the engine clamps it")
        self.assertTrue(state.prox_found_object, "the engine has no rule against a close object")


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Cliff(unittest.TestCase):
    def test_reaction_on_stops_backs_away_and_takes_control(self) -> None:
        sim = _edge_sim(CONTROL_DEFAULT)
        edge = 0.5 * sim.v["table_length_mm"]
        result = sim.drive_straight(300.0, 100.0)
        control_taken = not sim.has_control
        for _ in range(800):
            if sim.request_control():
                break
            sim.step(1)
        furthest = max(s.x_mm for s in sim.states)
        backed_when_the_reaction_ended = furthest - sim.pose_mm()[0]
        sim.advance(1.0)
        self.assertEqual(result.result, robot.RESULT_CLIFF)
        self.assertEqual(sim.reactions, 1)
        self.assertFalse(sim.sim_truth()["fallen"])
        self.assertLess(furthest, edge + _coast_mm(sim.v, 100.0), "the treads coast to a stop; no further")
        self.assertAlmostEqual(backed_when_the_reaction_ended, sim.v["cliff_reaction_backup_mm"], delta=3.0)
        self.assertGreater(furthest - sim.pose_mm()[0], sim.v["cliff_reaction_backup_mm"], "then it coasts a little more")
        self.assertTrue(any(s.cliff_detected for s in sim.states))
        self.assertFalse(any(s.falling for s in sim.states))
        self.assertTrue(control_taken, "the reaction takes control from the SDK")

    def test_after_a_reaction_commands_are_refused_until_control_is_requested(self) -> None:
        sim = _edge_sim(CONTROL_DEFAULT)
        sim.drive_straight(300.0, 100.0)
        sim.advance(1.0)
        self.assertEqual(sim.drive_straight(-20.0, 50.0).result, robot.RESULT_NO_CONTROL)
        self.assertEqual(sim.turn_in_place(90.0).result, robot.RESULT_NO_CONTROL)
        self.assertFalse(sim.set_wheel_motors(50.0, 50.0))
        self.assertTrue(sim.request_control())
        self.assertEqual(sim.drive_straight(-20.0, 50.0).result, robot.RESULT_COMPLETED)

    def test_control_cannot_be_taken_back_while_the_robot_is_still_reacting(self) -> None:
        sim = _edge_sim(CONTROL_DEFAULT)
        sim.set_wheel_motors(100.0, 100.0)
        for _ in range(600):
            sim.step(1)
            if sim.reactions:
                break
        self.assertEqual(sim.reactions, 1)
        self.assertFalse(sim.request_control())
        x_react = sim.pose_mm()[0]
        sim.advance(0.8 * sim.v["cliff_event_delay_ms"] / 1000.0)
        x_paused = sim.pose_mm()[0]
        self.assertGreaterEqual(x_paused, x_react, "it coasts and waits; it does not back away yet")
        sim.advance(2.0)
        self.assertLess(sim.pose_mm()[0], x_paused - 0.5 * sim.v["cliff_reaction_backup_mm"])
        self.assertTrue(sim.request_control())

    def test_reaction_off_the_robot_drives_off_and_falls(self) -> None:
        sim = _edge_sim(CONTROL_OVERRIDE)
        seen = []
        update = sim.hazards.update

        def watch(ranges: list, accel: float) -> None:
            update(ranges, accel)
            seen.append((accel, sim.hazards.cliff_any))

        sim.hazards.update = watch
        result = sim.drive_straight(300.0, 100.0)
        sim.advance(1.0)
        x, _, z = sim.pose_mm()
        self.assertEqual(sim.reactions, 0)
        self.assertTrue(sim.sim_truth()["fallen"])
        self.assertGreater(x, 0.5 * sim.v["table_length_mm"], "it left over the edge it drove at")
        self.assertLess(z, -0.5 * sim.v["table_height_mm"], "it reached the floor, not a ledge below the top")
        self.assertEqual(result.result, robot.RESULT_OFF_GROUND)
        self.assertTrue(any(s.cliff_detected for s in sim.states))
        # The drop itself, not the `falling` flag. The flag needs the free fall to last the firmware's 150 ms,
        # and how long the tumble's free fall lasts is chaotic: it held for 5 to 7 control ticks at an inset of
        # 3.1 mm, a state message carried it at 3.0 and not at 3.1, and on 2026-11-22 the same drop on an older
        # table stayed under the threshold for 3 state messages on Linux MuJoCo 4.0.1 against 5 on macOS 4.0.1
        # and 6 on 3.11.0. What is checked here is that the fall reached the detector's input: an acceleration
        # under the threshold while a cliff showed. The flag's rule is tested on known traces in
        # tests/test_hazards.py.
        low = sim.v["fall_accel_low_mmps2"]
        self.assertTrue(any(accel < low and cliff for accel, cliff in seen), "the free fall reaches the detector")
        self.assertTrue(any(s.picked_up for s in sim.states))
        self.assertTrue(sim.has_control, "at the override level the robot never takes control away")

    def test_reaction_holds_at_a_slow_speed(self) -> None:
        sim = _edge_sim(CONTROL_DEFAULT)
        result = sim.drive_straight(300.0, 50.0)
        sim.advance(1.0)
        self.assertEqual(result.result, robot.RESULT_CLIFF)
        self.assertFalse(sim.sim_truth()["fallen"])

    def test_backing_onto_a_rear_edge_reacts_by_driving_forward(self) -> None:
        v = params.values(params.load())
        sim = VectorSim(values=v, start_xy_mm=(-0.5 * v["table_length_mm"] + 120.0, 0.0))
        result = sim.drive_straight(-200.0, 60.0)
        sim.advance(1.0)
        lowest = min(s.x_mm for s in sim.states)
        self.assertEqual(result.result, robot.RESULT_CLIFF)
        self.assertFalse(sim.sim_truth()["fallen"])
        self.assertGreater(sim.pose_mm()[0], lowest + 0.5 * v["cliff_reaction_backup_mm"])


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Determinism(unittest.TestCase):
    @staticmethod
    def _scripted(sim: "VectorSim") -> list:
        sim.drive_straight(120.0, 100.0)
        sim.turn_in_place(60.0)
        sim.set_head_angle(20.0)
        sim.advance(0.3)
        return list(sim.states)

    def test_identical_seeds_give_identical_runs(self) -> None:
        first = self._scripted(VectorSim.randomised(7))
        second = self._scripted(VectorSim.randomised(7))
        self.assertGreater(len(first), 50)
        self.assertEqual(first, second)

    def test_a_different_seed_gives_a_different_run(self) -> None:
        self.assertNotEqual(self._scripted(VectorSim.randomised(7)), self._scripted(VectorSim.randomised(8)))

    def test_nominal_runs_repeat_exactly_including_a_fall(self) -> None:
        runs = []
        for _ in range(2):
            sim = _edge_sim(CONTROL_OVERRIDE)
            sim.drive_straight(300.0, 100.0)
            sim.advance(0.5)
            runs.append(list(sim.states))
        self.assertEqual(runs[0], runs[1])

    def test_robots_drawn_from_the_ranges_still_drive(self) -> None:
        for seed in range(1, 6):
            sim = VectorSim.randomised(seed)
            result = sim.drive_straight(100.0, 30.0)
            overrun = sim.pose_mm()[0] - 100.0
            self.assertEqual(result.result, robot.RESULT_COMPLETED, seed)
            self.assertGreater(overrun, -DRIVE_ALONG_TOL_MM, seed)
            self.assertLess(overrun, _coast_mm(sim.v, 30.0) + DRIVE_ALONG_TOL_MM, seed)


if __name__ == "__main__":
    unittest.main()
