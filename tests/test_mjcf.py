"""The generated MJCF follows the parameter table. Needs no MuJoCo.."""

from __future__ import annotations

import math
import unittest
import xml.etree.ElementTree as ET

import _common  # noqa: F401
from simulacrum import mjcf, params


def _find(root: ET.Element, tag: str, name: str) -> ET.Element:
    for elem in root.iter(tag):
        if elem.get("name") == name:
            return elem
    raise AssertionError(f"no <{tag} name={name!r}> in the model")


def _floats(text: str) -> list:
    return [float(x) for x in text.split()]


class GeneratedModel(unittest.TestCase):
    def setUp(self) -> None:
        self.doc = params.load()
        self.v = params.values(self.doc)
        self.root = ET.fromstring(mjcf.build_mjcf(self.v))

    def test_has_the_parts_the_robot_has(self) -> None:
        for body in ("chassis", "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr", "head", "lift_arm", "lift_fork"):
            _find(self.root, "body", body)
        for actuator in ("left_wheel", "right_wheel"):
            _find(self.root, "general", actuator)
        for actuator in ("head", "lift"):
            _find(self.root, "position", actuator)
        self.assertEqual(len(list(self.root.iter("rangefinder"))), 4)
        self.assertEqual(len(list(self.root.iter("gyro"))), 1)
        self.assertEqual(len(list(self.root.iter("accelerometer"))), 1)
        _find(self.root, "camera", "head_cam")

    def test_same_table_gives_the_same_text(self) -> None:
        self.assertEqual(mjcf.build_mjcf(self.v), mjcf.build_mjcf(params.values(self.doc)))

    def test_wheel_radius_comes_from_the_table(self) -> None:
        self.assertAlmostEqual(float(_find(self.root, "geom", "wheel_fl_geom").get("size")), 0.0145, places=9)
        changed = ET.fromstring(mjcf.build_mjcf(params.values(self.doc, {"wheel_front_radius_mm": 15.0})))
        self.assertAlmostEqual(float(_find(changed, "geom", "wheel_fl_geom").get("size")), 0.015, places=9)
        self.assertAlmostEqual(_floats(_find(changed, "body", "wheel_fl").get("pos"))[2], 0.015, places=9)

    def test_treads_sit_at_the_built_spacing_not_the_firmware_constant(self) -> None:
        changed = ET.fromstring(mjcf.build_mjcf(params.values(self.doc, {"tread_center_spacing_mm": 47.9,
                                                                            "tread_spacing_mm": 46.0})))
        self.assertAlmostEqual(_floats(_find(changed, "body", "wheel_fl").get("pos"))[1], 0.02395, places=9)
        self.assertAlmostEqual(_floats(_find(changed, "body", "wheel_rr").get("pos"))[1], -0.02395, places=9)

    def test_head_box_offset_moves_the_box_and_only_when_set(self) -> None:
        offset = self.v["head_box_center_wrt_neck_mm"]
        pos = _find(self.root, "geom", "head_geom").get("pos")
        self.assertIsNotNone(pos, "the head box was written without its offset")
        self.assertEqual(_floats(pos), [c * 0.001 for c in offset])
        centred = ET.fromstring(mjcf.build_mjcf(params.values(self.doc, {"head_box_center_wrt_neck_mm": [0.0, 0.0, 0.0]})))
        self.assertIsNone(_find(centred, "geom", "head_geom").get("pos"))

    def test_cube_contact_uses_its_outer_size(self) -> None:
        changed = ET.fromstring(mjcf.build_mjcf(params.values(self.doc, {"cube_outer_mm": 45.0}), (), "lift"))
        self.assertEqual(_floats(_find(changed, "geom", "cube_geom").get("size")), [0.0225] * 3)

    def test_the_wheels_and_tip_supports_take_the_tread_friction(self) -> None:
        for name in ("wheel_fl_geom", "wheel_rr_geom", "tip_front_l", "tip_rear_r"):
            friction = _floats(_find(self.root, "geom", name).get("friction"))
            self.assertAlmostEqual(friction[0], self.v["tread_friction"], places=9, msg=name)
        changed = ET.fromstring(mjcf.build_mjcf(params.values(self.doc, {"tread_friction": 0.6})))
        self.assertAlmostEqual(_floats(_find(changed, "geom", "wheel_fl_geom").get("friction"))[0], 0.6, places=9)

    def test_the_contact_inset_moves_the_wheels_inward_and_keeps_the_body_level(self) -> None:
        inset, base = self.v["tread_contact_inset_mm"], self.v["wheel_base_mm"]
        self.assertGreater(inset, 0.0)
        front = _floats(_find(self.root, "body", "wheel_fr").get("pos"))
        rear = _floats(_find(self.root, "body", "wheel_rl").get("pos"))
        self.assertAlmostEqual(front[0], -inset * 0.001, places=9)
        self.assertAlmostEqual(rear[0], -(base - inset) * 0.001, places=9)
        self.assertAlmostEqual(front[2], self.v["wheel_front_radius_mm"] * 0.001, places=9)
        self.assertAlmostEqual(rear[2], self.v["wheel_rear_radius_mm"] * 0.001, places=9)
        none = ET.fromstring(mjcf.build_mjcf(params.values(self.doc, {"tread_contact_inset_mm": 0.0})))
        self.assertAlmostEqual(_floats(_find(none, "body", "wheel_rr").get("pos"))[0], -base * 0.001, places=9)
        for bad in (-0.5, 0.25 * base):
            with self.assertRaises(AssertionError):
                mjcf.build_mjcf(params.values(self.doc, {"tread_contact_inset_mm": bad}))

    def test_every_ranged_robot_parameter_changes_the_model_or_is_used_by_the_layer(self) -> None:
        layer_only = {
            "cliff_detect_range_mm", "cliff_reaction_backup_mm", "cliff_reaction_backup_speed_mmps",
            "cliff_event_delay_ms", "command_delay_ms", "wheel_accel_default_mmps2",
            "wheel_decel_default_mmps2", "point_turn_accel_default_radps2", "heading_hold_gain_mmps_per_rad",
            "drive_done_mm", "wheel_speed_max_mmps", "servo_command_delay_ms", "head_speed_max_radps",
            "lift_speed_max_radps", "cliff_reaction_pause_ms", "gyro_noise_rms_dps", "gyro_bias_dps", "accel_noise_rms_mg", "accel_bias_mg", "accel_scale",
            "accel_filter_cutoff_hz", "gyro_filter_cutoff_hz", "imu_filter_second_pole",
            "tof_range_std_fraction", "battery_sag_v_per_power", "motor_reference_voltage_v",
            "tread_heat_tau_s", "tread_heat_loss_rate_per_s", "tof_quality_at_100mm", "tof_quality_distance_exponent",
            "tof_quality_floor", "tof_weak_distance_mm", "tof_weak_distance_sd_mm", "tof_lift_quality",
            "tof_lift_beam_window_mm",
            # the firmware's WHEEL_DIST_MM, in its turn arithmetic; the model puts the treads at tread_center_spacing_mm
            "tread_spacing_mm",
            # the carried cube's chip and app, run by simulacrum/cube.py
            "cube_accel_bandwidth_hz", "cube_accel_frame_period_ms", "cube_accel_filter_second_pole",
            "cube_accel_noise_density_ug", "cube_accel_noise_axis_scale", "cube_accel_bias_mg", "cube_accel_scale"}
        unchanged = []
        for name, entry in self.doc["parameters"].items():
            if entry["kind"] != "robot" or "low" not in entry or name in layer_only:
                continue
            mount = "lift" if name.startswith("cube_") else None  # the cube's body is only in a model carrying it
            if mjcf.build_mjcf(params.values(self.doc, {name: entry["high"]}), cube=mount) == mjcf.build_mjcf(self.v, cube=mount):
                unchanged.append(name)
        self.assertEqual(unchanged, [])

    def test_table_size_comes_from_the_table(self) -> None:
        size = _floats(_find(self.root, "geom", "table").get("size"))
        self.assertAlmostEqual(size[0], 0.5 * self.v["table_length_mm"] / 1000.0, places=9)
        self.assertAlmostEqual(size[1], 0.5 * self.v["table_width_mm"] / 1000.0, places=9)
        wide = ET.fromstring(mjcf.build_mjcf(params.values(self.doc, {"table_length_mm": 900.0})))
        self.assertAlmostEqual(_floats(_find(wide, "geom", "table").get("size"))[0], 0.45, places=9)
        floor = _floats(_find(self.root, "geom", "floor").get("pos"))
        self.assertAlmostEqual(floor[2], -self.v["table_height_mm"] / 1000.0, places=9)

    def test_cliff_sensors_sit_where_the_table_says_in_manual_order(self) -> None:
        v = self.v
        expected = {"front_left": (v["cliff_front_x_mm"], v["cliff_y_mm"]),
                    "front_right": (v["cliff_front_x_mm"], -v["cliff_y_mm"]),
                    "back_left": (v["cliff_rear_x_mm"], v["cliff_y_mm"]),
                    "back_right": (v["cliff_rear_x_mm"], -v["cliff_y_mm"])}
        names = [s.get("name") for s in self.root.iter("rangefinder")][:4]
        self.assertEqual(names, ["cliff_" + n for n in v["cliff_sensor_order"]])
        for name, (x_mm, y_mm) in expected.items():
            pos = _floats(_find(self.root, "site", "cliff_" + name).get("pos"))
            self.assertAlmostEqual(pos[0] * 1000.0, x_mm, places=6)
            self.assertAlmostEqual(pos[1] * 1000.0, y_mm, places=6)
            self.assertAlmostEqual(pos[2] * 1000.0, v["cliff_sensor_height_mm"], places=6)
            self.assertEqual(_floats(_find(self.root, "site", "cliff_" + name).get("zaxis")), [0.0, 0.0, -1.0])

    def test_time_of_flight_site_is_tilted_up_by_the_table_angle(self) -> None:
        site = _find(self.root, "site", "tof")
        axis = _floats(site.get("zaxis"))
        self.assertAlmostEqual(math.degrees(math.atan2(axis[2], axis[0])), self.v["tof_tilt_up_deg"], places=6)
        self.assertEqual([round(c * 1000.0, 6) for c in _floats(site.get("pos"))], self.v["tof_position_mm"])

    def test_joint_limits_and_speed_limit_come_from_the_table(self) -> None:
        head = _floats(_find(self.root, "joint", "head_joint").get("range"))
        self.assertAlmostEqual(math.degrees(head[0]), self.v["head_angle_min_deg"], places=6)
        self.assertAlmostEqual(math.degrees(head[1]), self.v["head_angle_max_deg"], places=6)
        lift = _floats(_find(self.root, "joint", "lift_joint").get("range"))
        self.assertAlmostEqual(mjcf.lift_height_of_angle(self.v, lift[0]), self.v["lift_height_min_mm"], places=6)
        self.assertAlmostEqual(mjcf.lift_height_of_angle(self.v, lift[1]), self.v["lift_height_max_mm"], places=6)
        self.assertEqual(_floats(_find(self.root, "general", "left_wheel").get("ctrlrange")), [-1.0, 1.0])

    def test_tread_motor_is_the_linear_motor_the_table_describes(self) -> None:
        """Held at a steady speed the motor needs power = speed / no-load speed + friction share."""
        v = self.v
        motor = _find(self.root, "general", "left_wheel")
        stall = float(motor.get("gainprm"))
        drag = -_floats(motor.get("biasprm"))[2]
        friction = float(_find(self.root, "joint", "wheel_fl_joint").get("frictionloss"))
        self.assertAlmostEqual(stall * 1000.0, v["wheel_stall_torque_mNm"], places=9)
        for speed in (60.0, 120.0, 220.0):
            wheel = speed / v["wheel_front_radius_mm"]
            power = (drag * wheel + friction) / stall
            line = speed / v["tread_noload_speed_mmps"] + v["tread_friction_power_fraction"]
            self.assertAlmostEqual(power, line, places=9, msg=str(speed))
        fric_tc = self.v["control_period_ms"] * 0.001
        for side in ("wheel_fl_joint", "wheel_fr_joint"):
            raw = _find(self.root, "joint", side).get("solreffriction")
            self.assertIsNotNone(raw)
            self.assertEqual(_floats(raw), [fric_tc, 1.0])
        self.assertIsNone(_find(self.root, "joint", "wheel_rl_joint").get("frictionloss"))
        self.assertIsNone(_find(self.root, "joint", "wheel_rl_joint").get("solreffriction"))
        self.assertIsNone(_find(self.root, "joint", "wheel_rr_joint").get("solreffriction"))
        cp6 = ET.fromstring(mjcf.build_mjcf(params.values(self.doc, {"control_period_ms": 6.0})))
        self.assertEqual(_floats(_find(cp6, "joint", "wheel_fl_joint").get("solreffriction")), [0.006, 1.0])

    def test_dock_ramp_and_pocket_preserve_friction_while_funnels_stay_soft(self) -> None:
        root = ET.fromstring(mjcf.build_mjcf(self.v, dock="docked"))
        tc = self.v["contact_time_constant_ms"] * 0.001
        for name in ("dock_lip", "dock_apron", "dock_bed"):
            geom = _find(root, "geom", name)
            self.assertEqual(geom.get("priority"), "2", name)
            self.assertAlmostEqual(_floats(geom.get("friction"))[0], self.v["tread_friction"], places=9, msg=name)
            self.assertEqual(_floats(geom.get("solref")), [tc, 1.0], name)
            self.assertEqual(_floats(geom.get("solimp")), [0.999, 0.999, 0.001], name)
        pocket = _find(root, "geom", "dock_pocket")
        self.assertEqual(pocket.get("priority"), "2")
        self.assertAlmostEqual(_floats(pocket.get("friction"))[0], 0.05, places=9)
        self.assertEqual(_floats(pocket.get("solref")), [tc, 1.0])
        self.assertEqual(_floats(pocket.get("solimp")), [0.999, 0.999, 0.001])
        funnel = _find(root, "geom", "dock_funnel_l")
        self.assertIsNone(funnel.get("solref"))
        self.assertIsNone(funnel.get("solimp"))

    def test_part_masses_add_up_to_the_total(self) -> None:
        masses = mjcf.derive(self.v)["masses_g"]
        tip = 0.04 if mjcf._tip_supports_driven(self.v) else 0.0
        total = masses["chassis"] + masses["head"] + masses["arm"] + masses["fork"] + 4.0 * masses["wheel"] + tip
        self.assertAlmostEqual(total, self.v["total_mass_g"], places=9)
        with self.assertRaises(AssertionError):
            mjcf.derive(params.values(self.doc, {"head_mass_g": 400.0}))

    def test_timestep_must_divide_the_control_period(self) -> None:
        with self.assertRaises(AssertionError):
            mjcf.derive(params.values(self.doc, {"sim_timestep_ms": 0.7}))

    def test_obstacles_are_placed_in_millimetres(self) -> None:
        root = ET.fromstring(mjcf.build_mjcf(self.v, [(150.0, 0.0, 50.0, 10.0, 200.0, 100.0)]))
        box = _find(root, "geom", "obstacle_0")
        self.assertEqual([round(c, 9) for c in _floats(box.get("pos"))], [0.15, 0.0, 0.05])
        self.assertEqual([round(c, 9) for c in _floats(box.get("size"))], [0.005, 0.1, 0.05])

    def test_free_cube_shell_uses_contact_time_constant_and_is_stiffer_than_rubber(self) -> None:
        root = ET.fromstring(mjcf.build_mjcf(self.v, (), "free"))
        tc = self.v["contact_time_constant_ms"] * 0.001
        shells = [g for g in root.iter("geom") if g.get("name", "").startswith("cube_shell_")]
        rubbers = [g for g in root.iter("geom") if g.get("name", "").startswith("cube_rubber_")]
        self.assertGreater(len(shells), 0)
        self.assertEqual(len(rubbers), 8)
        for geom in shells:
            self.assertIsNotNone(geom.get("solref"), geom.get("name"))
            solref = _floats(geom.get("solref"))
            self.assertEqual(solref, [tc, 1.0], geom.get("name"))
        for geom in rubbers:
            self.assertLess(tc, _floats(geom.get("solref"))[0], geom.get("name"))
        changed = ET.fromstring(mjcf.build_mjcf(params.values(self.doc, {"contact_time_constant_ms": 6.0}), (), "free"))
        shell0 = [g for g in changed.iter("geom") if g.get("name", "").startswith("cube_shell_")][0]
        self.assertEqual(_floats(shell0.get("solref")), [0.006, 1.0])

    @unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
    def test_build_rescue_mjcf_has_two_robots_and_optional_cube_and_exclusions(self) -> None:
        import mujoco

        v = params.values(self.doc)
        direct_xml = mjcf.build_rescue_mjcf(v, with_cube=False)
        direct_model = mujoco.MjModel.from_xml_string(direct_xml)
        self.assertEqual(direct_model.nu, 8)
        self.assertGreaterEqual(mujoco.mj_name2id(direct_model, mujoco.mjtObj.mjOBJ_BODY, "patient_chassis"), 0)
        self.assertEqual(mujoco.mj_name2id(direct_model, mujoco.mjtObj.mjOBJ_BODY, "cube"), -1)

        cube_xml = mjcf.build_rescue_mjcf(v, with_cube=True)
        cube_model = mujoco.MjModel.from_xml_string(cube_xml)
        self.assertEqual(cube_model.nu, 8)
        self.assertGreaterEqual(mujoco.mj_name2id(cube_model, mujoco.mjtObj.mjOBJ_EQUALITY, "carry_weld"), 0)
        self.assertNotIn('body1="chassis" body2="patient_chassis"', cube_xml)

        excl_xml = mjcf.build_rescue_mjcf(v, with_cube=False, exclude_robot_contacts=True)
        self.assertIn('body1="chassis" body2="patient_chassis"', excl_xml)


if __name__ == "__main__":
    unittest.main()
