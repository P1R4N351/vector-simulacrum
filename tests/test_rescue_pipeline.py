"""Tests for experiments/righting/rescue_pipeline.py self-check and scene wiring."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import _common

SDK_CUBE_PATH = _common.ROOT / "web" / "public" / "robot" / "sdk" / "cube.obj"


def _load_rescue_pipeline():
    righting_dir = _common.ROOT / "experiments" / "righting"
    module_path = righting_dir / "rescue_pipeline.py"
    assert module_path.is_file(), f"missing pipeline script: {module_path}"
    spec = importlib.util.spec_from_file_location("righting_rescue_pipeline", module_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(righting_dir))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(righting_dir))
    return module


class RescuePipelineTests(unittest.TestCase):
    @unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
    def test_paired_check_rejects_identical_and_accepts_divergent(self) -> None:
        rp = _load_rescue_pipeline()
        with tempfile.TemporaryDirectory() as tmp:
            tdir = Path(tmp)
            a = tdir / "a.json"
            b = tdir / "b.json"
            c = tdir / "c.json"
            base = {"sides": {"right": {"final_patient_tilt_deg": 0.0}, "left": {"final_patient_tilt_deg": 0.0}}}
            diff = {"sides": {"right": {"final_patient_tilt_deg": 90.02}, "left": {"final_patient_tilt_deg": 90.02}}}
            a.write_text(json.dumps(base), encoding="utf-8")
            b.write_text(json.dumps(base), encoding="utf-8")
            c.write_text(json.dumps(diff), encoding="utf-8")
            bad = rp.paired_check(a, b)
            self.assertFalse(bad["ok"])
            self.assertEqual(bad["differing_fields"], [])
            good = rp.paired_check(a, c)
            self.assertTrue(good["ok"])
            self.assertEqual(len(good["differing_fields"]), 2)

    @unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
    def test_scene_xml_excludes_robot_contacts_when_requested(self) -> None:
        rp = _load_rescue_pipeline()
        candidates = [[[[0.0, 0.0, 0.0], [0.022, 0.022, 0.022]]]]
        if SDK_CUBE_PATH.is_file():
            candidates.append(rp.collision_boxes(SDK_CUBE_PATH))
        for boxes in candidates:
            xml_on = rp.scene_xml(boxes, 50.0, 0.5, 0.9, False, with_cube=True)
            xml_off = rp.scene_xml(boxes, 50.0, 0.5, 0.9, True, with_cube=True)
            self.assertIn("carry_weld", xml_on)
            self.assertNotIn('body1="chassis" body2="patient_chassis"', xml_on)
            self.assertIn('body1="chassis" body2="patient_chassis"', xml_off)

    @unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
    def test_continuous_nocube_arm_contact_on_vs_off_diverges(self) -> None:
        rp = _load_rescue_pipeline()
        model_on = rp.mj.MjModel.from_xml_string(rp.scene_xml(None, 50.0, 0.5, 0.9, False, with_cube=False))
        model_off = rp.mj.MjModel.from_xml_string(rp.scene_xml(None, 50.0, 0.5, 0.9, True, with_cube=False))
        args = rp.parse_args(["/tmp/unused", "--no-cube"])
        f_on, log_on = rp.run_one(model_on, 1, args)
        f_off, log_off = rp.run_one(model_off, 1, args)
        self.assertGreater(len(f_on), 100)
        self.assertGreater(len(f_off), 100)
        self.assertLess(log_on["nocube_final_patient_tilt_deg"], 5.0)
        self.assertGreater(log_off["nocube_final_patient_tilt_deg"], 85.0)
        self.assertTrue(log_on["nocube_rescuer_clear_at_end"])
