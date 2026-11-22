"""The check that a run used the pinned MuJoCo.

    python3 tests/run_all.py --only test_mujoco_pin

simulacrum/mujoco_pin.py decides from the library mapped into the process,
not only from the version string, because the GPU host's own package directory
holds files of two MuJoCo versions at once. These tests make the check fail
on purpose: a check that cannot fail would let a run on the wrong model pass.
"""

from __future__ import annotations

import os
import unittest
from pathlib import Path
from unittest import mock

import _common
from simulacrum import mujoco_pin


class ExpectedVersion(unittest.TestCase):
    def test_the_pin_is_the_browser_build_version(self) -> None:
        with mock.patch.dict(os.environ, {mujoco_pin.EXPECT_ENV: ""}):
            self.assertEqual(mujoco_pin.expected_version(), "4.0.1")

    def test_an_override_is_used_and_a_malformed_one_refused(self) -> None:
        with mock.patch.dict(os.environ, {mujoco_pin.EXPECT_ENV: "3.11.0"}):
            self.assertEqual(mujoco_pin.expected_version(), "3.11.0")
        with mock.patch.dict(os.environ, {mujoco_pin.EXPECT_ENV: "3.11"}):
            with self.assertRaises(ValueError):
                mujoco_pin.expected_version()


@unittest.skipUnless(_common.HAVE_MUJOCO, "needs the mujoco package")
class LoadedLibrary(unittest.TestCase):
    def setUp(self) -> None:
        import mujoco  # noqa: PLC0415

        self.loaded = mujoco.mj_versionString()

    def test_the_loaded_version_passes_and_is_read_from_the_mapped_library(self) -> None:
        result = mujoco_pin.check(self.loaded)
        self.assertTrue(result["ok"], result["problems"])
        if result["libmujoco"] is not None:
            self.assertEqual(len(result["libmujoco"]), 1)
            self.assertIn(self.loaded, result["libmujoco"][0])

    def test_another_version_fails_on_both_the_bindings_and_the_library(self) -> None:
        other = "3.11.0" if self.loaded != "3.11.0" else "4.0.1"
        result = mujoco_pin.check(other)
        self.assertFalse(result["ok"])
        self.assertTrue(any("bindings report" in p for p in result["problems"]))
        if result["libmujoco"] is not None:
            self.assertTrue(any(p.startswith("loaded ") for p in result["problems"]))

    def test_a_library_of_another_version_fails_even_when_the_bindings_agree(self) -> None:
        other = "3.11.0" if self.loaded != "3.11.0" else "4.0.1"
        import mujoco  # noqa: PLC0415

        # Inside the imported package, as in the GPU host's mixed directory, so only the version can tell.
        stray = str(Path(mujoco.__file__).resolve().parent / f"libmujoco.so.{other}")
        with mock.patch.object(mujoco_pin, "loaded_libmujoco", return_value=[stray]):
            result = mujoco_pin.check(self.loaded)
        self.assertEqual(result["bindings"], self.loaded, "the bindings agree; only the library is wrong")
        self.assertFalse(result["ok"])
        self.assertTrue(any(stray in p for p in result["problems"]))

    def test_the_right_version_from_another_package_fails(self) -> None:
        elsewhere = f"/opt/another-venv/site-packages/mujoco/libmujoco.so.{self.loaded}"
        with mock.patch.object(mujoco_pin, "loaded_libmujoco", return_value=[elsewhere]):
            result = mujoco_pin.check(self.loaded)
        self.assertFalse(result["ok"])
        self.assertTrue(any("outside the imported package" in p for p in result["problems"]))

    def test_no_library_found_is_a_failure_not_a_pass(self) -> None:
        with mock.patch.object(mujoco_pin, "loaded_libmujoco", return_value=[]):
            result = mujoco_pin.check(self.loaded)
        self.assertFalse(result["ok"])


if __name__ == "__main__":
    unittest.main()
