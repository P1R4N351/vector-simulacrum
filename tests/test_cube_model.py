"""The light cube's accelerometer model (simulacrum/cube.py) and the cube carried in the lift..

The first three classes need numpy only. Their expected values are built apart from the code they check:
packets with struct from protocol.h's layout, frame values coded by the time they were taken, tap sequences
written out by hand from accel.c's rules, and the noise level from the chip filter's own impulse response.
The last two classes need MuJoCo and say what the model does, not what a real cube does.
"""

from __future__ import annotations

import importlib.util
import math
import os
import struct
import unittest
from pathlib import Path
from typing import Any, Callable, Dict, List
from unittest import mock

import numpy as np

import _common
from simulacrum import cube, imu_filter, params

V = params.values(params.load())
QUIET = dict(V, cube_accel_noise_density_ug=0.0)
OPEN = dict(QUIET, cube_accel_bandwidth_hz=1.0e6)  # a filter that passes every step's value unchanged
STEP = 0.001
LSB = 512.0  # counts per g of the 12-bit value at +/-4 g (BMA253DS:384)


def run(v: Dict[str, Any], seconds: float, force: Callable[[int], List[float]], **kwargs: Any) -> cube.CubeAccelerometer:
    """Step a cube for whole milliseconds; force(n) is the specific force (g) at the end of step n."""
    chip = cube.CubeAccelerometer(v, STEP, **kwargs)
    chip.reset(np.array(force(0), dtype=float), 0.0)
    for n in range(1, int(round(seconds / STEP)) + 1):
        chip.step(n * STEP, np.array(force(n), dtype=float))
    return chip


def unpack(data: bytes) -> List[int]:
    fields = struct.unpack("<BB9h", data)
    assert fields[0] == 2, "not an accelerometer message"
    return list(fields[2:])


class ChipAndApp(unittest.TestCase):
    def test_at_rest_on_its_base_the_cube_sends_one_g_on_z_every_30_ms(self) -> None:
        chip = run(QUIET, 1.0, lambda n: [0.0, 0.0, 1.0])
        self.assertEqual(len(chip.messages), 33)
        self.assertEqual([round(t, 6) for t, _ in chip.messages[:2]], [0.03, 0.06])
        for _, data in chip.messages:
            self.assertEqual(len(data), 20)
            self.assertEqual(unpack(data), [0, 0, 8192] * 3)

    def test_values_are_twelve_bits_left_justified_and_clipped_at_four_g(self) -> None:
        chip = run(QUIET, 0.03, lambda n: [5.0, -5.0, 0.5])
        self.assertEqual(unpack(chip.messages[0][1])[:3], [2047 * 16, -2048 * 16, 256 * 16])
        chip = run(QUIET, 0.03, lambda n: [1.0 / LSB * 0.49, -3.0 / LSB, 0.0])
        self.assertEqual(unpack(chip.messages[0][1])[:3], [0, -3 * 16, 0])

    def test_unit3_stock_packet_word_spacing_and_clipping(self):
        values = dict(OPEN, cube_accel_word_format=params.values(params.load(model="vector_2"))["cube_accel_word_format"])
        chip = run(values, .03, lambda n: [-5.0, 5.0, -4/8192])
        self.assertEqual(unpack(chip.messages[0][1]), [-32767, 32765, -3]*3)
        decoded = cube.samples_g(cube.rows(chip.messages), 8192, padding_bits=2)
        np.testing.assert_equal(decoded[0]*8192, [-32768, 32764, -4])
        mean_chip = run(values, .03, lambda n: [0.0, 0.0, 1.0], app="mean")
        self.assertEqual(unpack(mean_chip.messages[0][1]), [0, 0, 8192]*3)

    def test_each_tick_keeps_the_last_frame_it_reads_and_a_frame_due_with_the_tick_is_read(self) -> None:
        coded = lambda n: [n / LSB, 0.0, 0.0]  # step n reads n counts on x
        chip = run(OPEN, 0.03, coded)
        self.assertEqual(unpack(chip.messages[0][1])[0::3], [10 * 16, 20 * 16, 30 * 16])
        chip = run(OPEN, 0.03, coded, frame_phase_s=0.0005)  # frames at 2.5, 4.5, ... ms land on steps 3, 5, ...
        self.assertEqual(unpack(chip.messages[0][1])[0::3], [9 * 16, 19 * 16, 29 * 16])

    def test_the_mean_app_keeps_the_mean_of_each_ticks_frames(self) -> None:
        coded = lambda n: [n / LSB, 0.0, 0.0]  # step n reads n counts on x; frames every 2 ms
        chip = run(OPEN, 0.03, coded, app="mean")
        self.assertEqual(unpack(chip.messages[0][1])[0::3], [6 * 16, 16 * 16, 26 * 16])  # frames 2..10, 12..20, 22..30
        steady = run(QUIET, 0.3, lambda n: [0.0, 0.0, 1.0], app="mean")
        self.assertEqual([unpack(d) for _, d in steady.messages], [unpack(d) for _, d in run(QUIET, 0.3, lambda n: [0.0, 0.0, 1.0]).messages])

    def test_a_late_tick_finds_a_full_fifo_and_keeps_an_old_frame(self) -> None:
        late = dict(OPEN, cube_tick_ms=100.0)  # 50 frames per tick against a FIFO of 32
        chip = run(late, 0.3, lambda n: [n / LSB, 0.0, 0.0])
        self.assertEqual(unpack(chip.messages[0][1])[0::3], [64 * 16, 164 * 16, 264 * 16])
        self.assertEqual(chip.dropped, 3 * (50 - 32))

    def test_a_tick_with_no_frame_leaves_its_slot_as_it_was(self) -> None:
        slow = dict(OPEN, cube_accel_frame_period_ms=25.0)
        chip = run(slow, 0.06, lambda n: [n / LSB, 0.0, 0.0])
        self.assertEqual(unpack(chip.messages[0][1])[0::3], [0, 0, 25 * 16])
        self.assertEqual(unpack(chip.messages[1][1])[0::3], [0, 50 * 16, 25 * 16])

    def test_noise_comes_out_at_the_density_through_the_filter(self) -> None:
        density_g = V["cube_accel_noise_density_ug"] * 1e-6
        chip = run(V, 20.0, lambda n: [0.0, 0.0, 1.0], seed=7)
        samples = cube.samples_g(cube.rows(chip.messages), V["cube_accel_counts_per_g"])
        self.assertEqual(samples.shape, (len(chip.messages) * 3, 3))
        # Expected: white noise of one-sided density n up to the step's Nyquist, through this filter, quantised.
        probe = imu_filter.ImuFilter(V["cube_accel_bandwidth_hz"], V["cube_accel_bandwidth_hz"],
                                     V["cube_accel_filter_second_pole"], STEP)
        impulse = []
        for k in range(400):
            probe.update(np.array([1.0 if k == 0 else 0.0] + [0.0] * 5))
            impulse.append(float(probe.output[0]))
        sigma_in = density_g * math.sqrt(0.5 / STEP)
        expected = math.sqrt(sigma_in ** 2 * sum(h * h for h in impulse) + (1.0 / LSB) ** 2 / 12.0)
        for axis in range(3):
            self.assertAlmostEqual(float(samples[:, axis].std()) / expected, 1.0, delta=0.08, msg=str(axis))
        self.assertAlmostEqual(float(samples[:, 2].mean()), 1.0, delta=0.002)

    def test_reading_drops_the_four_low_bits_a_real_chip_may_fill(self) -> None:
        rows = [{"k": "a", "t": 0.03, "tap": 0, "raw": [8192 + 15, -8192 + 9, 1, 0, 0, 0, 16, -1, -16]}]
        got = cube.samples_g(rows, V["cube_accel_counts_per_g"])
        self.assertEqual(got[0].tolist(), [1.0, -1.0, 0.0])
        self.assertEqual(got[2].tolist(), [16 / 8192, -16 / 8192, -16 / 8192])

    def test_messages_read_the_same_through_the_collector(self) -> None:
        spec = importlib.util.spec_from_file_location("cube_accel_collector", _common.ROOT / "hardware" / "cube_accel.py")
        assert spec is not None and spec.loader is not None, "cannot load the collector"
        with mock.patch.dict(os.environ, {"CUBE_MODE": "scan"}):
            collector = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(collector)
        chip = run(V, 0.3, lambda n: [0.1, -0.2, 1.0], seed=3)
        for row, (t, data) in zip(cube.rows(chip.messages), chip.messages):
            kind, fields = collector.decode(data)
            self.assertEqual((kind, fields["raw"], fields["tap"]), ("a", row["raw"], row["tap"]))


class Taps(unittest.TestCase):
    @staticmethod
    def count(zs: List[int]) -> int:
        counter = cube.TapCounter()
        for z in zs:
            counter.feed(z)
        return counter.count

    def test_rest_and_the_first_frame_are_not_taps(self) -> None:
        self.assertEqual(self.count([8192] * 200), 0)  # the first frame rises 8192 from zero and starts a window

    def test_a_sharp_rise_and_fall_within_ten_frames_is_a_tap(self) -> None:
        self.assertEqual(self.count([8192] * 100 + [8192 + 3000] + [8192] * 100), 1)
        self.assertEqual(self.count([8192] * 100 + [8192 + 3000] * 5 + [8192] * 100), 1)

    def test_a_rise_that_falls_back_too_late_or_too_little_is_not(self) -> None:
        self.assertEqual(self.count([8192] * 100 + [8192 + 3000] * 12 + [8192] * 100), 0)
        self.assertEqual(self.count([8192] * 100 + [8192 + 3000] + [8192 + 500] * 100), 0)
        self.assertEqual(self.count([8192] * 100 + [8192 + 2880] + [8192] * 100), 0)  # the threshold is exclusive

    def test_as_shipped_a_sharp_drop_counts_at_the_next_frame_that_does_not_rise_by_90(self) -> None:
        low = 8192 - 3000
        self.assertEqual(self.count([8192] * 100 + [low] * 100), 1)
        self.assertEqual(self.count([8192] * 100 + [low, low + 90] + [low + 90] * 100), 1)  # one rise, then flat
        rising = [low + 90 * k for k in range(1, 11)]  # every frame of the ten-frame window rises by 90
        self.assertEqual(self.count([8192] * 100 + [low] + rising + [rising[-1]] * 100), 0)

    def test_the_count_wraps_at_256_and_a_long_window_counts_once(self) -> None:
        tap = [8192 + 3000, 8192] + [8192] * 100
        self.assertEqual(self.count([8192] * 100 + tap * 257), 1)  # 257 taps wrap to 1
        self.assertEqual(self.count([8192] * 100 + [8192 + 3000, 8192, 8192 + 3000, 8192] + [8192] * 100), 1)


class TurnCentre(unittest.TestCase):
    def test_the_centre_comes_back_from_samples_built_for_it(self) -> None:
        g, omega, centre = V["gravity_mmps2"], 4.0, np.array([-40.0, 10.0])
        rng = np.random.default_rng(5)
        still = np.array([0.01, -0.02, 1.0]) + rng.normal(0.0, 0.004, (300, 3))
        pull = np.concatenate([omega * omega * centre / g, [0.0]])
        spinning = np.array([0.01, -0.02, 1.0]) + pull + rng.normal(0.0, 0.004, (300, 3))
        got = cube.turn_centre(still, spinning, omega, g)
        self.assertLess(float(np.hypot(*(np.array(got["centre_mm"]) - centre))), 0.5)
        self.assertAlmostEqual(got["radius_mm"], float(np.hypot(*centre)), delta=0.5)

    def test_a_chip_carried_round_a_centre_reports_it_through_the_whole_chain(self) -> None:
        g_mm, omega, centre_mm = V["gravity_mmps2"], 4.0, np.array([-45.0, 8.0])
        still = run(V, 1.0, lambda n: [0.0, 0.0, 1.0], seed=11)
        pull = list(omega * omega * centre_mm / g_mm) + [1.0]  # the chip's axes turn with the body
        spun = run(V, 3.0, lambda n: pull if n > 0 else [0.0, 0.0, 1.0], seed=12)
        got = cube.turn_centre(cube.samples_g(cube.rows(still.messages), V["cube_accel_counts_per_g"]),
                               cube.samples_g(cube.rows(spun.messages[20:]), V["cube_accel_counts_per_g"]),
                               omega, g_mm)
        self.assertLess(float(np.hypot(*(np.array(got["centre_mm"]) - centre_mm))), 1.0)


class MeanPacketPrecision(unittest.TestCase):
    def test_mean_app_retains_signed_fractional_counts(self):
        row = {"raw": [3, -3, 8195] * 3}
        samples = cube.samples_g([row], 8192, app="mean")
        np.testing.assert_equal(samples[0] * 8192, [3, -3, 8195])


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class FreeCube(unittest.TestCase):
    def test_push_moves_independent_cube_seen_by_both_sensors(self):
        from simulacrum.robot import VectorSim
        sim = VectorSim(values=QUIET, cube="free", cube_app="mean")
        sim.advance(.5)
        start = sim.data.body("cube").xpos.copy()
        self.assertTrue(sim._prox().found_object)
        before = len(sim.cube.messages)
        self.assertTrue(sim.set_wheel_motors(40, 40))
        sim.advance(2.3)
        self.assertGreater(sim.data.body("cube").xpos[0] - start[0], .01)
        messages = sim.cube.messages[before:]
        self.assertGreaterEqual(len(messages), 75)
        samples = cube.samples_g(cube.rows(messages), QUIET["cube_accel_counts_per_g"], app="mean")
        self.assertGreater(np.max(np.abs(samples[:, :2])), .05)
        self.assertTrue(any(int(c) % 16 for row in cube.rows(messages) for c in row["raw"]))


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class CarriedCube(unittest.TestCase):
    def test_the_model_carries_the_cube_where_the_engine_puts_it(self) -> None:
        import mujoco
        from simulacrum import mjcf
        plain = mujoco.MjModel.from_xml_string(mjcf.build_mjcf(V))
        carried = mujoco.MjModel.from_xml_string(mjcf.build_mjcf(V, cube="lift"))
        self.assertEqual(mujoco.mj_name2id(plain, mujoco.mjtObj.mjOBJ_BODY, "cube"), -1)
        self.assertAlmostEqual(float(carried.body_subtreemass[1] - plain.body_subtreemass[1]),
                               V["cube_mass_g"] * 0.001, places=9)
        data = mujoco.MjData(carried)
        mujoco.mj_forward(carried, data)
        fork = mujoco.mj_name2id(carried, mujoco.mjtObj.mjOBJ_BODY, "lift_fork")
        body = mujoco.mj_name2id(carried, mujoco.mjtObj.mjOBJ_BODY, "cube")
        offset = data.xmat[fork].reshape(3, 3).T @ (data.xpos[body] - data.xpos[fork])
        self.assertTrue(np.allclose(offset * 1000.0, V["cube_carry_offset_mm"], atol=1e-6), str(offset))

    def test_held_still_at_carry_height_the_chip_reads_one_g_up(self) -> None:
        from simulacrum.robot import VectorSim
        sim = VectorSim(values=HOLDABLE, cube="lift", noise_seed=4)
        self.assertAlmostEqual(sim.lift_height_mm(), V["lift_height_max_mm"], delta=2.0)
        sim.advance(1.0)
        self.assertAlmostEqual(sim.lift_height_mm(), V["lift_height_max_mm"], delta=2.0)
        self.assertIsNotNone(sim.cube)
        samples = cube.samples_g(cube.rows(sim.cube.messages), V["cube_accel_counts_per_g"])
        self.assertGreaterEqual(len(sim.cube.messages), 32)
        mean = samples.mean(axis=0)
        self.assertLess(float(np.hypot(mean[0], mean[1])), 0.02, str(mean))
        self.assertAlmostEqual(float(mean[2]), 1.0, delta=0.02)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class SpinWithTheCube(unittest.TestCase):
    """Turn in place carrying the cube and find the turn centre the way the spin test with the real cube would,
    against where the model's chip actually circles.

    Two checks. The force robot.py feeds the chip, averaged over every physics step, must give the centre to
    SPIN_TOLERANCE_MM: that tests the carry pose, the chip's frame, the feed and the arithmetic. The cube's own
    samples give it only to within their standard error: the model's chip feels about 0.6 g RMS of tread
    vibration in this turn (evidence/cube-model-20261121/alias-output.txt), which the app's one frame per tick
    aliases without any filter, so two seconds of samples leave tens of mg. That check is weak by nature; the
    chain from force to samples is tested exactly by the classes above."""

    WHEEL_MMPS = 80.0
    STILL_S, SPIN_UP_S, MEASURE_S = 1.0, 2.0, 2.0

    def test_the_force_at_the_chip_gives_the_turn_centre_and_the_samples_agree_within_their_error(self) -> None:
        import mujoco
        from simulacrum.robot import VectorSim
        sim = VectorSim(values=HOLDABLE, cube="lift", noise_seed=6)
        assert sim.cube is not None
        forces: List[np.ndarray] = []
        inner = sim.cube.step
        sim.cube.step = lambda t, f: (forces.append(np.array(f, dtype=float)), inner(t, f))[1]
        sim.advance(self.STILL_S)
        still_f, still_m = len(forces), len(sim.cube.messages)
        sim.set_wheel_motors(-self.WHEEL_MMPS, self.WHEEL_MMPS)
        sim.advance(self.SPIN_UP_S)
        first_f, first_m = len(forces), len(sim.cube.messages)
        site = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_SITE, "cube_chip")
        path, frames, yaws, times = [], [], [], []
        for _ in range(int(round(self.MEASURE_S / sim.dt))):
            sim.step(1)
            path.append(np.array(sim.data.site_xpos[site][:2]))
            frames.append(np.array(sim.data.site_xmat[site]).reshape(3, 3))
            yaws.append(sim.heading_total_rad())
            times.append(float(sim.data.time))
        omega = (yaws[-1] - yaws[0]) / (times[-1] - times[0])
        self.assertGreater(abs(omega), 2.0, "the robot did not turn")
        # Where the chip circles: a least-squares circle through its path, then that centre on the chip's axes.
        pts = np.array(path)
        a = np.column_stack([2.0 * pts, np.ones(len(pts))])
        cx, cy, _ = np.linalg.lstsq(a, (pts ** 2).sum(axis=1), rcond=None)[0]
        truth = np.mean([(f.T @ np.array([cx - p[0], cy - p[1], 0.0]))[:2] for f, p in zip(frames, pts)], axis=0) * 1000.0
        fed = cube.turn_centre(np.array(forces[:still_f]), np.array(forces[first_f:]), omega, V["gravity_mmps2"])
        self.assertLess(float(np.hypot(*(np.array(fed["centre_mm"]) - truth))), SPIN_TOLERANCE_MM,
                        "the fed force says %s mm, the chip circles %s mm" % (fed["centre_mm"], truth.tolist()))
        cpg = V["cube_accel_counts_per_g"]
        s_still = cube.samples_g(cube.rows(sim.cube.messages[:still_m]), cpg)
        s_spin = cube.samples_g(cube.rows(sim.cube.messages[first_m:]), cpg)
        got = cube.turn_centre(s_still, s_spin, omega, V["gravity_mmps2"])
        error_g = np.sqrt(s_still.var(axis=0) / len(s_still) + s_spin.var(axis=0) / len(s_spin))[:2]
        error_mm = error_g * V["gravity_mmps2"] / (omega * omega)
        miss = np.abs(np.array(got["centre_mm"]) - truth)
        self.assertTrue(np.all(miss < 4.0 * error_mm), "samples miss by %s mm, their standard error is %s mm"
                        % (miss.round(1).tolist(), error_mm.round(1).tolist()))


class TapListenerTests(unittest.TestCase):
    def test_first_count_jump_wrap_and_repeat(self):
        listener = cube.TapListener()
        for t, count in enumerate([249, 249, 255, 0, 0, 1]):
            listener.receive(t, cube.packet(count, [[0, 0, 0]] * 3))
        self.assertEqual(listener.events, [2, 3, 5])

    def test_packet_stream_and_reset(self):
        chip = run(OPEN, .9, lambda n: [0, 0, 2 if (n//20)%2 else -2])
        expected = [t for (_, a), (t, b) in zip(chip.messages, chip.messages[1:]) if a[1] != b[1]]
        self.assertTrue(expected)
        self.assertEqual(chip.taps.events, expected)
        chip.reset(np.array([0, 0, 1]), 0.)
        self.assertEqual(chip.taps.events, [])


class MovementTests(unittest.TestCase):
    def test_threshold_and_hysteresis(self):
        listener = cube.MovementListener()
        listener.update(0., np.zeros(3, dtype=np.float32))
        for tick in range(1, 52):
            accel = np.array([5000 * (-1)**tick, 0, 0], dtype=np.float32)
            listener.update(tick / 100, accel)
            if tick == 50:
                self.assertEqual(listener.score, 10000.)
                self.assertFalse(listener.moving)
        self.assertEqual(listener.events, [(.51, True)])
        for tick in range(52, 200):
            listener.update(tick / 100, accel)
        self.assertFalse(listener.moving)
        self.assertEqual([state for _, state in listener.events], [True, False])

    def test_handling_filters_broadcast_at_transition_only(self):
        listener = cube.MovementListener()
        for tick in range(80):
            accel = np.array([5000 * (-1)**tick, 0, 0], dtype=np.float32)
            listener.update(tick / 100, accel, handling_cube=True)
        self.assertTrue(listener.moving)
        self.assertEqual(listener.broadcast_events, [])
        for tick in range(80, 200):
            listener.update(tick / 100, accel)
        self.assertEqual(listener.broadcast_events, listener.events[1:])
        self.assertEqual([state for _, state in listener.broadcast_events], [False])

    def test_packet_delivery_and_reset(self):
        chip = run(OPEN, 1.2, lambda n: [(-1)**(n//10), 0, 1])
        self.assertTrue(chip.movement.moving)
        self.assertEqual(len(chip.movement.events), 1)
        self.assertIn(chip.movement.events[0][0], [t for t, _ in chip.messages])
        chip.reset(np.array([0,0,1]), 0.)
        self.assertEqual(chip.movement.events, [])
        self.assertEqual(chip.movement.score, 0.)


class UpAxisTests(unittest.TestCase):
    def test_filter_delay_initial_silence_and_engine_codes(self):
        # An instantaneous quarter turn crosses at the 23rd 10 ms reading:
        # the old axis decays as .97**n; the new one grows as 1-.97**n.
        for app in cube.APPS:
            chip = run(OPEN, .60, lambda n: [0, 0, 1] if n <= 300 else [-1, 0, 0], app=app)
            self.assertEqual(chip.up_axis.events, [(.54, 1)])
            chip.reset(np.array([0, 0, 1]), 0)
            self.assertEqual(chip.up_axis.events, [])
            self.assertEqual(chip.up_axis.axis, 0)

    def test_all_faces_and_tie_order(self):
        listener = cube.UpAxisListener()
        t = 0.0
        for value, axis in [([8000,8000,0],2), ([0,0,-8000],5), ([0,-8000,0],3),
                            ([-8000,0,0],1), ([0,0,8000],6), ([0,8000,0],4)]:
            for _ in range(40):
                t += .03
                listener.receive(t, cube.packet(0, [value]*3))
            self.assertEqual(listener.axis, axis)
        self.assertEqual([axis for _, axis in listener.events], [5,3,1,6,4])


SPIN_TOLERANCE_MM = 5.0
HOLDABLE = dict(V, cube_mass_g=25.0)  # the table's 50 g falls out of its assumed 30 mN*m lift; see README


if __name__ == "__main__":
    unittest.main()
