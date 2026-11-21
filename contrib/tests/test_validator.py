"""Validator self-test. Runs against two fixtures under contrib/tests/fixtures/:
a good submission that must pass every check, and a bad one that carries a planted
IP address and must fail the leak scan."""
from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
FIXTURES = HERE / "fixtures"


def _load_validator():
    """Load contrib/validator.py by path so the test runs from any cwd."""
    spec = importlib.util.spec_from_file_location("validator", HERE.parent / "validator.py")
    assert spec and spec.loader, "could not import contrib.validator"
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class ValidatorTests(unittest.TestCase):
    """Three cases: the good fixture validates clean, the bad fixture fails on the leak,
    and a fixture whose folder name fails HANDLE_RX is reported."""

    def setUp(self) -> None:
        self.v = _load_validator()
        self.good = FIXTURES / "fixture-good-1"
        self.bad = FIXTURES / "fixture-bad-1"
        self.assertTrue(self.good.is_dir())
        self.assertTrue(self.bad.is_dir())

    def test_good_fixture_validates(self) -> None:
        problems = self.v.validate(self.good)
        self.assertEqual(problems, [], f"expected no problems, got {problems}")

    def test_bad_fixture_fails_on_leak(self) -> None:
        problems = self.v.validate(self.bad)
        self.assertTrue(any("IPv4" in p for p in problems),
                        f"expected IPv4 leak flagged, got {problems}")

    def test_handle_regex_rejects_bare_name(self) -> None:
        self.assertFalse(self.v.HANDLE_RX.match("example-handle"))
        self.assertTrue(self.v.HANDLE_RX.match("example-1"))


if __name__ == "__main__":
    unittest.main()
