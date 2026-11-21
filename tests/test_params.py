"""The parameter table and its checks. Needs no MuJoCo.."""

from __future__ import annotations

import copy
import unittest

import _common  # noqa: F401  (puts the skill on the path)
from simulacrum import params


class Vector2(unittest.TestCase):
    def setUp(self) -> None:
        self.one, self.two = params.load(model="vector_1"), params.load(model="vector_2")

    def test_it_replaces_only_what_it_names_and_marks_inherited_measurements(self) -> None:
        one, two = self.one["parameters"], self.two["parameters"]
        self.assertEqual(set(one), set(two))
        replaced = {n for n, e in two.items() if e.get("overrides_base")}
        self.assertEqual(replaced, {"display_resolution_px", "battery_capacity_mah", "total_mass_g", "com_x_mm",
                                    "com_z_mm", "camera_resolution_px", "camera_fov_horizontal_deg", "camera_fov_vertical_deg",
                                    "accel_scale", "accel_bias_mg", "imu_pitch_offset_deg"})
        self.assertEqual((one["display_resolution_px"]["value"], two["display_resolution_px"]["value"]), ([184, 96], [160, 80]))
        for name in set(one) - replaced:
            self.assertEqual(one[name]["value"], two[name]["value"], name)
            inherited_measurement = one[name]["provenance"] in ("measured", "fitted")
            self.assertEqual("inherited" in two[name], inherited_measurement, name)
            self.assertNotIn("inherited", one[name], name)
        self.assertEqual(params.summary(self.two)["inherited"], 27)
        self.assertEqual(self.two["robot"], "Digital Dream Labs Vector 2.0")

    def test_unit3_camera_uses_its_reported_intrinsics_at_stream_resolution(self) -> None:
        from web.tools.export_model import camera_block
        camera = camera_block(params.values(self.two))
        self.assertEqual((camera["width_px"], camera["height_px"]), (800, 600))
        self.assertAlmostEqual(camera["fx_px"], 406.314453)
        self.assertAlmostEqual(camera["fy_px"], 405.335419)
        self.assertGreater(camera["cx_px"], 400)
        self.assertLess(camera["cy_px"], 300)

    def test_ranges_for_what_2_0_may_change_cover_the_1_0_ranges(self) -> None:
        for name in ("total_mass_g", "com_x_mm", "com_z_mm"):
            a, b = self.one["parameters"][name], self.two["parameters"][name]
            self.assertLessEqual(b["low"], a["low"], name)
            self.assertGreaterEqual(b["high"], a["high"], name)

    def test_the_model_comes_from_the_environment_and_bad_names_are_refused(self) -> None:
        import os
        from unittest import mock
        with mock.patch.dict(os.environ, {params.MODEL_ENV: "vector_2"}):
            self.assertEqual(params.load()["model"], "vector_2")
        with mock.patch.dict(os.environ, {params.MODEL_ENV: "vector_3"}), self.assertRaises(ValueError):
            params.load()

    def test_a_variant_naming_an_unknown_entry_is_refused(self) -> None:
        import json
        base = json.loads(params.TABLE_PATH.read_text(encoding="utf-8"))
        variant = json.loads(params.MODELS["vector_2"].read_text(encoding="utf-8"))
        variant["overrides"]["no_such_thing"] = dict(variant["overrides"]["total_mass_g"])
        with self.assertRaises(ValueError):
            params._apply_variant(base, variant)
        clash = json.loads(params.MODELS["vector_2"].read_text(encoding="utf-8"))
        clash["sources"]["TRM"] = {"what": "something else"}
        with self.assertRaises(ValueError):
            params._apply_variant(base, clash)


class TableIsSound(unittest.TestCase):
    def setUp(self) -> None:
        self.doc = params.load()

    def test_table_loads_without_problems(self) -> None:
        self.assertEqual(params.validate(self.doc), [])
        self.assertGreaterEqual(len(self.doc["parameters"]), 90)

    def test_every_entry_has_one_allowed_provenance(self) -> None:
        for name, entry in self.doc["parameters"].items():
            self.assertIn(entry["provenance"], params.PROVENANCE, name)
            self.assertTrue(entry["source"].strip(), name)

    def test_counts_add_up_and_measured_and_fitted_cite_the_recording(self) -> None:
        counts = params.summary(self.doc)
        self.assertEqual((counts["measured"], counts["fitted"], counts["assumed"], counts["chosen"]), (10, 19, 38, 8))
        self.assertEqual(sum(counts[name] for name in params.PROVENANCE), counts["total"])
        self.assertGreater(counts["missing"], 0)
        for name, entry in self.doc["parameters"].items():
            if entry["provenance"] in ("measured", "fitted"):
                self.assertIn("REC", entry["source"], name)
            if entry["provenance"] == "fitted":
                self.assertTrue("Fitted by fit_recording.py" in entry["source"]
                                or "simulacrum/imu_fit.py" in entry["source"]
                                or "simulacrum/motor_heat_fit.py" in entry["source"]
                                or "simulacrum/prox_fit.py" in entry["source"]
                                or "simulacrum/spin_fit.py" in entry["source"], name)
                self.assertIn("low", entry, name)

    def test_every_assumed_entry_has_a_range_around_its_value(self) -> None:
        for name, entry in self.doc["parameters"].items():
            if entry["provenance"] != "assumed":
                continue
            self.assertIn("low", entry, name)
            self.assertIn("high", entry, name)

    def test_the_table_describes_exactly_the_classes_the_code_allows(self) -> None:
        described = self.doc["provenance_classes"]
        self.assertEqual(sorted(described), sorted(params.PROVENANCE))
        counts = params.summary(self.doc)
        for name, text in described.items():
            self.assertGreater(len(text.strip()), 40, name)
            self.assertFalse("No entry" in text and counts[name] > 0, name)

    def test_missing_list_says_how_to_measure(self) -> None:
        for item in self.doc["missing"]:
            for key in ("name", "what", "measure", "needs"):
                self.assertTrue(item[key], item)


class ValidatorCatchesBadTables(unittest.TestCase):
    def setUp(self) -> None:
        self.doc = copy.deepcopy(params.load())
        self.entry = self.doc["parameters"]["total_mass_g"]

    def test_control_unchanged_copy_is_valid(self) -> None:
        self.assertEqual(params.validate(self.doc), [])

    def test_unknown_provenance_is_rejected(self) -> None:
        self.entry["provenance"] = "guessed"
        self.assertTrue(any("provenance" in p for p in params.validate(self.doc)))

    def test_assumed_without_range_is_rejected(self) -> None:
        del self.entry["low"]
        del self.entry["high"]
        self.assertTrue(any("assumed without low/high" in p for p in params.validate(self.doc)))

    def test_value_outside_its_range_is_rejected(self) -> None:
        self.entry["value"] = self.entry["high"] + 1.0
        self.assertTrue(any("not inside" in p for p in params.validate(self.doc)))

    def test_sourced_entry_must_cite_a_defined_source(self) -> None:
        self.doc["parameters"]["wheel_base_mm"]["source"] = "somewhere"
        self.assertTrue(any("cites no key" in p for p in params.validate(self.doc)))

    def test_sdk_entry_needs_a_source_kind(self) -> None:
        del self.doc["parameters"]["wheel_base_mm"]["source_kind"]
        self.assertTrue(any("source_kind" in p for p in params.validate(self.doc)))

    def test_a_fit_may_not_be_filed_as_a_measurement_without_the_recording(self) -> None:
        self.entry["provenance"] = "measured"
        self.assertTrue(any("must cite the recording" in p for p in params.validate(self.doc)))
        fitted = self.doc["parameters"]["command_delay_ms"]
        del fitted["low"]
        del fitted["high"]
        self.assertTrue(any("fitted without low/high" in p for p in params.validate(self.doc)))

    def test_a_class_the_table_does_not_describe_is_rejected(self) -> None:
        del self.doc["provenance_classes"]["fitted"]
        self.assertTrue(any("provenance_classes" in p for p in params.validate(self.doc)))

    def test_only_scene_and_solver_settings_may_be_filed_as_chosen(self) -> None:
        self.doc["parameters"]["total_mass_g"]["provenance"] = "chosen"
        self.assertTrue(any("total_mass_g" in p and "chosen" in p for p in params.validate(self.doc)))

    def test_a_scene_setting_may_not_be_filed_as_an_assumption_about_the_robot(self) -> None:
        self.doc["parameters"]["table_length_mm"]["provenance"] = "assumed"
        self.assertTrue(any("table_length_mm" in p and "chosen" in p for p in params.validate(self.doc)))

    def test_empty_source_is_rejected(self) -> None:
        self.entry["source"] = "  "
        self.assertTrue(any("empty source" in p for p in params.validate(self.doc)))


class ValuesAndSampling(unittest.TestCase):
    def setUp(self) -> None:
        self.doc = params.load()

    def test_override_changes_one_value_only(self) -> None:
        base = params.values(self.doc)
        changed = params.values(self.doc, {"total_mass_g": 160.0})
        self.assertEqual(changed["total_mass_g"], 160.0)
        self.assertEqual({k for k in base if base[k] != changed[k]}, {"total_mass_g"})

    def test_unknown_override_is_refused(self) -> None:
        with self.assertRaises(KeyError):
            params.values(self.doc, {"no_such_parameter": 1.0})

    def test_sample_stays_inside_every_range(self) -> None:
        for seed in range(20):
            drawn = params.sample(self.doc, seed)
            for name, entry in self.doc["parameters"].items():
                if "low" not in entry or entry["kind"] != "robot":
                    self.assertEqual(drawn[name], entry["value"], name)
                    continue
                lows = entry["low"] if isinstance(entry["low"], list) else [entry["low"]]
                highs = entry["high"] if isinstance(entry["high"], list) else [entry["high"]]
                vals = drawn[name] if isinstance(drawn[name], list) else [drawn[name]]
                for low, val, high in zip(lows, vals, highs):
                    self.assertTrue(low <= val <= high, (name, seed, val))

    def test_same_seed_same_sample_and_other_seed_differs(self) -> None:
        self.assertEqual(params.sample(self.doc, 11), params.sample(self.doc, 11))
        self.assertNotEqual(params.sample(self.doc, 11), params.sample(self.doc, 12))


if __name__ == "__main__":
    unittest.main()
