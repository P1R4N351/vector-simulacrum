"""The tread heat fit, on a constructed run and on REC2 run F; and the model under sustained load.."""

from __future__ import annotations

import json
import math
import statistics
import tempfile
import unittest
from pathlib import Path
from typing import Any, Dict, List

import _common
from simulacrum import motor_heat_fit, params

RUN_F = Path(__file__).resolve().parent.parent / "evidence" / "hw-rec-20261118" / "run-f-endurance-unit1.jsonl"


def constructed_run(tau: float, loss: float, cycles: int = 15) -> List[Dict[str, Any]]:
    """An endurance run whose right tread loses `loss` of its ceiling at full heat with time constant tau;
    the left tread holds. 3 s spins both ways, 2 s rests, state every 60 ms."""
    rows: List[Dict[str, Any]] = []
    t, h, dt = 0.0, 0.0, 0.06
    for cycle in range(cycles):
        for way, sign in (("ccw", 1.0), ("cw", -1.0)):
            rows.append({"k": "c", "t": round(t, 4), "name": "wheels", "label": f"endurance_{way}_{cycle}",
                         "left": -sign * 300.0, "right": sign * 300.0, "seconds": 3.0})
            for on, seconds in ((True, 3.0), (False, 2.0)):
                if not on:
                    rows.append({"k": "c", "t": round(t, 4), "name": "stop", "why": "segment_end"})
                for _ in range(int(round(seconds / dt))):
                    h += ((1.0 if on else 0.0) - h) * (1.0 - math.exp(-dt / tau))
                    right_cold = 250.0 if sign > 0 else 244.0
                    speed = (right_cold * (1.0 - loss * h), 246.0) if on else (0.0, 0.0)
                    rows.append({"k": "s", "t": round(t, 4), "wl": -sign * speed[1], "wr": sign * speed[0]})
                    t += dt
    return rows


def _write(rows: List[Dict[str, Any]], folder: str) -> Path:
    path = Path(folder) / "run.jsonl"
    path.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    return path


class ConstructedRun(unittest.TestCase):
    def test_a_known_heating_is_recovered_and_a_steady_tread_is_not_given_one(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            points = motor_heat_fit.spins(_write(constructed_run(60.0, 0.15), tmp))
        self.assertEqual(len(points), 30)
        right = motor_heat_fit.fit_tread(points, "right")
        self.assertAlmostEqual(right["loss_at_full_heat"] / right["tau_s"], 0.15 / 60.0, delta=0.3 * 0.15 / 60.0)
        self.assertLess(right["rmse_held_out_mmps"], right["rmse_held_out_constant_mmps"])
        left = motor_heat_fit.fit_tread(points, "left")
        self.assertLess(abs(left["a_mmps"]), 0.5)

    def test_spins_ended_by_a_hazard_are_left_out(self) -> None:
        rows = constructed_run(60.0, 0.15, cycles=12)
        last_stop = max(i for i, r in enumerate(rows) if r.get("name") == "stop")
        rows[last_stop]["why"] = "cliff_detected"
        with tempfile.TemporaryDirectory() as tmp:
            points = motor_heat_fit.spins(_write(rows, tmp))
        self.assertEqual(len(points), 23)


class RunF(unittest.TestCase):
    def test_the_right_tread_heats_and_the_left_does_not(self) -> None:
        points = motor_heat_fit.spins(RUN_F)
        self.assertEqual(len(points), 29)
        right = motor_heat_fit.fit_tread(points, "right")
        left = motor_heat_fit.fit_tread(points, "left")
        self.assertLess(right["rmse_held_out_mmps"], 0.5 * right["rmse_held_out_constant_mmps"])
        self.assertGreater(left["rmse_held_out_mmps"], left["rmse_held_out_constant_mmps"])
        rate = right["loss_at_full_heat"] / right["tau_s"]
        table = params.values(params.load())["tread_heat_loss_rate_per_s"]
        self.assertAlmostEqual(table[1], rate, delta=0.0001)
        self.assertEqual(table[0], 0.0)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class ModelUnderLoad(unittest.TestCase):
    def test_sustained_full_power_spins_slow_the_right_tread_as_fitted(self) -> None:
        from simulacrum.robot import VectorSim
        v = params.values(params.load())
        sim = VectorSim(values=v)
        ceilings: Dict[str, List[float]] = {"left": [], "right": []}
        for cycle in range(15):
            for sign in (1.0, -1.0):
                sim.set_wheel_motors(-sign * 300.0, sign * 300.0)
                sim.step(int(round(1.5 / sim.dt)))
                speeds = []
                for _ in range(int(round(1.5 / sim.dt))):
                    sim.step(1)
                    speeds.append(sim.reported_now())
                ceilings["left"].append(statistics.fmean(abs(s["wl"]) for s in speeds))
                ceilings["right"].append(statistics.fmean(abs(s["wr"]) for s in speeds))
                sim.stop_all_motors()
                sim.step(int(round(2.0 / sim.dt)))
        fall = {side: 1.0 - statistics.fmean(c[-4:]) / statistics.fmean(c[:2]) for side, c in ceilings.items()}
        # The fit's own prediction for the right tread over the same schedule, as a fraction of its cold ceiling.
        points = motor_heat_fit.spins(RUN_F)
        right = motor_heat_fit.fit_tread(points, "right")
        heats = motor_heat_fit.heat_means(points, right["tau_s"])
        predicted = right["a_mmps"] * (statistics.fmean(heats[-4:]) - statistics.fmean(heats[:2])) / (
            0.5 * (right["c_forward_mmps"] + right["c_back_mmps"]))
        self.assertLess(abs(fall["left"]), 0.01)
        self.assertGreater(fall["right"], 0.5 * predicted)
        self.assertLess(fall["right"], 1.5 * predicted)


if __name__ == "__main__":
    unittest.main()
