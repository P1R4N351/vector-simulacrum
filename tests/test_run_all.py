"""The runner's verdict on skips (tests/run_all.py).."""

from __future__ import annotations

import unittest

import _common
import run_all


class _Probe(unittest.TestCase):
    def test_probe(self) -> None:
        pass


def result_with(reasons: list) -> unittest.TestResult:
    result = unittest.TestResult()
    result.testsRun = len(reasons) + 1
    for reason in reasons:
        result.addSkip(_Probe("test_probe"), reason)
    return result


class Verdict(unittest.TestCase):
    def test_a_skip_for_an_optional_input_passes_under_require_mujoco(self) -> None:
        result = result_with([_common.IMAGE_SKIP_REASON, _common.CC_SKIP_REASON])
        self.assertEqual(run_all.verdict(result, 3, True), (True, 2))

    def test_any_other_skip_fails_under_require_mujoco_and_passes_without_it(self) -> None:
        result = result_with([_common.IMAGE_SKIP_REASON, _common.SKIP_REASON])
        self.assertEqual(run_all.verdict(result, 3, True), (False, 1))
        self.assertEqual(run_all.verdict(result, 3, False), (True, 1))

    def test_a_count_short_of_the_declared_fails(self) -> None:
        self.assertEqual(run_all.verdict(result_with([]), 2, False), (False, 0))


if __name__ == "__main__":
    unittest.main()
