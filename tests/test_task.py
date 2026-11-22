"""The training task: what the policy sees, what it may do, how episodes end..

Needs MuJoCo, not torch.
"""

from __future__ import annotations

import math
import types
import unittest

import _common
from simulacrum import params

if _common.HAVE_MUJOCO:
    import numpy as np
    from simulacrum import task
    from simulacrum.task import TaskConfig, VectorTask
    from simulacrum.vec_env import MAX_WORKERS, TaskPool, rollout

REPORTED = ("left_wheel_speed_mmps", "right_wheel_speed_mmps", "gyro_radps", "accel_mmps2", "cliff_detected",
            "prox_distance_mm", "prox_found_object", "head_angle_rad", "lift_height_mm")
NOMINAL = TaskConfig(randomise=False, override_fraction=0.0) if _common.HAVE_MUJOCO else None


def _reported_only_state() -> types.SimpleNamespace:
    """A state message that carries only what the robot reports. Anything else raises AttributeError."""
    return types.SimpleNamespace(
        left_wheel_speed_mmps=110.0, right_wheel_speed_mmps=-55.0, gyro_radps=(0.1, -0.2, 0.5),
        accel_mmps2=(0.0, 981.0, 9810.0), cliff_detected=True, prox_distance_mm=300.0,
        prox_found_object=True, head_angle_rad=math.radians(22.5), lift_height_mm=62.0)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Observation(unittest.TestCase):
    def setUp(self) -> None:
        self.v = params.values(params.load())
        self.scales = task.observation_scales(self.v)

    def test_observation_uses_only_what_the_robot_reports(self) -> None:
        self.assertEqual(len(_reported_only_state().__dict__), len(REPORTED))
        try:
            obs = task.build_observation(_reported_only_state(), (80.0, 60.0), (0.25, -0.5), self.scales)
        except AttributeError as err:
            self.fail(f"the observation reads something the robot does not report: {err}")
        self.assertEqual(len(obs), task.OBS_DIM)
        self.assertEqual(len(task.OBS_NAMES), 17)
        self.assertFalse(any(word in name for name in task.OBS_NAMES for word in ("pose", "x_mm", "y_mm", "fall")))

    def test_observation_values_follow_the_table_scales(self) -> None:
        obs = task.build_observation(_reported_only_state(), (80.0, 60.0), (0.25, -0.5), self.scales)
        expected = (0.5, -0.25, 0.1 / math.radians(500.0), -0.2 / math.radians(500.0), 0.5 / math.radians(500.0),
                    0.0, 0.1, 1.0, 1.0, 0.25, 1.0, 0.5, 0.5, 0.5, 0.5, 0.25, -0.5)
        for name, got, want in zip(task.OBS_NAMES, obs, expected):
            self.assertAlmostEqual(got, want, places=9, msg=name)

    def test_first_observation_comes_from_the_robot_state_message(self) -> None:
        env = VectorTask(4, NOMINAL)
        obs = env.observation()
        state = env.sim.state
        self.assertAlmostEqual(obs[7], state.accel_mmps2[2] / self.v["gravity_mmps2"], places=9)
        self.assertAlmostEqual(obs[11], state.head_angle_rad / math.radians(self.v["head_angle_max_deg"]), places=9)
        self.assertAlmostEqual(obs[13], env.command[0] / self.v["directive_forward_speed_max_mmps"], places=9)
        self.assertEqual(obs[15:], (0.0, 0.0))


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Action(unittest.TestCase):
    def test_action_is_two_wheel_speeds_inside_the_table_limit(self) -> None:
        v = params.values(params.load())
        self.assertEqual(task.wheel_speeds_of_action((2.0, -3.0), v), (220.0, -220.0))
        self.assertEqual(task.wheel_speeds_of_action((0.5, 0.25), v), (110.0, 55.0))
        env = VectorTask(1, NOMINAL)
        env.step((5.0, -0.5))
        self.assertEqual(env.sim._wheel_target, [220.0, -110.0])
        self.assertEqual(env.last_action, (1.0, -0.5))

    def test_policy_step_is_one_state_period(self) -> None:
        env = VectorTask(1, NOMINAL)
        v = env.nominal
        self.assertAlmostEqual(env.step_s * 1000.0, v["state_period_ms"], places=9)
        tick0, states0 = env.sim.tick, len(env.sim.states)
        env.step((0.1, 0.1))
        self.assertEqual((env.sim.tick - tick0) * v["control_period_ms"], v["state_period_ms"])
        self.assertEqual(len(env.sim.states) - states0, 1)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Randomisation(unittest.TestCase):
    def setUp(self) -> None:
        self.doc = params.load()

    def test_every_assumed_parameter_but_the_timestep_is_varied(self) -> None:
        assumed = {n for n, e in self.doc["parameters"].items() if e["provenance"] in ("assumed", "chosen")}
        varied = set(task.varied_parameters(self.doc))
        self.assertEqual(assumed - varied, {"sim_timestep_ms"})
        identified = {n for n, e in self.doc["parameters"].items()
                      if e["provenance"] in ("measured", "fitted") and "low" in e}
        self.assertEqual(identified - varied, set(), "measured and fitted values are varied inside their ranges too")
        env = VectorTask(9)
        unchanged = [name for name in sorted(varied) if env.values[name] == env.nominal[name]]
        self.assertEqual(unchanged, [], "these kept their table value in a drawn robot")
        self.assertEqual(env.values["sim_timestep_ms"], env.nominal["sim_timestep_ms"])

    def test_drawn_robots_stay_inside_the_ranges_and_differ_between_episodes(self) -> None:
        env = VectorTask(2)
        seen = []
        for _ in range(8):
            env.reset()
            seen.append(env.values["total_mass_g"])
            for name in task.varied_parameters(self.doc):
                entry = self.doc["parameters"][name]
                lows = entry["low"] if isinstance(entry["low"], list) else [entry["low"]]
                highs = entry["high"] if isinstance(entry["high"], list) else [entry["high"]]
                vals = env.values[name] if isinstance(env.values[name], list) else [env.values[name]]
                for low, val, high in zip(lows, vals, highs):
                    self.assertTrue(low <= val <= high, name)
        self.assertEqual(len(set(seen)), 8)

    def test_randomisation_can_be_switched_off(self) -> None:
        env = VectorTask(2, NOMINAL)
        self.assertEqual(env.values, env.nominal)

    def test_commands_stay_inside_the_directive_limits(self) -> None:
        env = VectorTask(3, NOMINAL)
        v = env.nominal
        draws = [env._draw_command() for _ in range(400)]
        self.assertTrue(all(0.0 <= f <= v["directive_forward_speed_max_mmps"] for f, _ in draws))
        self.assertTrue(all(abs(t) <= v["directive_turn_speed_max_dps"] for _, t in draws))
        self.assertTrue(any(f == 0.0 and t == 0.0 for f, t in draws), "a stand-still command must occur")
        self.assertGreater(max(f for f, _ in draws), 0.8 * v["directive_forward_speed_max_mmps"])


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Episode(unittest.TestCase):
    @staticmethod
    def _drive_to_the_end(env: "VectorTask", action: tuple) -> list:
        results = []
        for _ in range(env.max_steps + 1):
            results.append(env.step(action))
            if results[-1].terminated or results[-1].truncated:
                break
        return results

    def test_a_fall_ends_the_episode_as_a_failure_with_a_penalty(self) -> None:
        env = VectorTask(5, TaskConfig(randomise=False, override_fraction=1.0, command_resample_s=100.0))
        env.command = (110.0, 0.0)
        last = self._drive_to_the_end(env, (0.5, 0.5))[-1]
        self.assertEqual(last.end_reason, task.END_FALL)
        self.assertTrue(last.terminated)
        self.assertFalse(last.truncated)
        self.assertLess(last.reward, -0.5 * env.config.fall_penalty)

    def test_a_cliff_reaction_ends_the_episode_without_blaming_the_policy(self) -> None:
        env = VectorTask(5, TaskConfig(randomise=False, override_fraction=0.0, command_resample_s=100.0))
        env.command = (66.0, 0.0)
        last = self._drive_to_the_end(env, (0.3, 0.3))[-1]
        self.assertEqual(last.end_reason, task.END_REACTION)
        self.assertTrue(last.truncated)
        self.assertFalse(last.terminated)
        self.assertGreater(last.reward, -1.0)
        self.assertFalse(env.sim.sim_truth()["fallen"])

    def test_the_time_limit_truncates(self) -> None:
        env = VectorTask(5, TaskConfig(randomise=False, override_fraction=0.0, episode_s=0.6))
        results = self._drive_to_the_end(env, (0.0, 0.0))
        self.assertEqual(len(results), 10)
        self.assertEqual(results[-1].end_reason, task.END_TIME)
        self.assertTrue(results[-1].truncated and not results[-1].terminated)

    def test_reward_is_highest_when_the_command_is_met(self) -> None:
        env = VectorTask(5, NOMINAL)
        env.command = (100.0, 30.0)
        met = env._reward(100.0, 30.0, env.last_action)[0]
        self.assertAlmostEqual(met, env.config.weight_forward + env.config.weight_turn, places=9)
        self.assertGreater(met, env._reward(100.0, -30.0, env.last_action)[0] + 0.2)
        self.assertGreater(met, env._reward(40.0, 30.0, env.last_action)[0] + 0.2)
        self.assertGreater(met, env._reward(100.0, 30.0, (1.0, -1.0))[0])

    def test_following_the_command_scores_above_standing_still(self) -> None:
        nominal = params.values(params.load())

        def kinematic(obs: "np.ndarray") -> "np.ndarray":
            rows = [task.kinematic_action((row[13] * nominal["directive_forward_speed_max_mmps"],
                                           row[14] * nominal["directive_turn_speed_max_dps"]), nominal)
                    for row in obs]
            return np.array(rows, dtype=np.float32)

        scores = {}
        for name, policy in (("kinematic", kinematic), ("still", lambda obs: np.zeros((obs.shape[0], 2)))):
            pool = TaskPool(4, seed=3, workers=0, config=NOMINAL)
            scores[name] = rollout(pool, policy, 80)
            pool.close()
        self.assertGreater(scores["kinematic"]["reward_per_step"], scores["still"]["reward_per_step"] + 0.3)
        self.assertEqual(scores["kinematic"]["env_steps"], 320)

    def test_same_seed_same_episode_and_another_seed_differs(self) -> None:
        def run(seed: int) -> list:
            env = VectorTask(seed)
            return [env.step((0.3, 0.2)).observation for _ in range(25)] + [tuple(sorted(env.values.items()))[:5]]

        self.assertEqual(run(11), run(11))
        self.assertNotEqual(run(11), run(12))


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Pool(unittest.TestCase):
    def test_worker_processes_do_not_change_the_result(self) -> None:
        serial, split = TaskPool(4, seed=5, workers=0), TaskPool(4, seed=5, workers=2)
        try:
            self.assertTrue(np.array_equal(serial.obs, split.obs))
            for _ in range(20):
                action = np.full((4, 2), 0.25, dtype=np.float32)
                a, b = serial.step(action), split.step(action)
                self.assertTrue(np.array_equal(a["obs"], b["obs"]))
                self.assertTrue(np.array_equal(a["reward"], b["reward"]))
        finally:
            serial.close()
            split.close()

    def test_more_than_four_worker_processes_is_refused(self) -> None:
        self.assertEqual(MAX_WORKERS, 4)
        with self.assertRaises(AssertionError):
            TaskPool(8, seed=0, workers=5)

    def test_pool_seed_decides_the_environments(self) -> None:
        a, b, c = TaskPool(3, seed=1), TaskPool(3, seed=1), TaskPool(3, seed=2)
        self.assertTrue(np.array_equal(a.obs, b.obs))
        self.assertFalse(np.array_equal(a.obs, c.obs))
        self.assertEqual(len({tuple(row) for row in a.obs}), 3, "environments in one pool must differ")

    def test_a_finished_environment_is_reset_and_reported(self) -> None:
        pool = TaskPool(2, seed=4, workers=0, config=TaskConfig(randomise=False, override_fraction=0.0, episode_s=0.3))
        episodes = []
        for _ in range(5):
            out = pool.step(np.zeros((2, 2), dtype=np.float32))
            episodes.extend(out["episodes"])
        self.assertEqual(len(episodes), 2)
        self.assertEqual({record[0] for record in episodes}, {task.END_TIME})
        self.assertTrue(out["truncated"].all())
        self.assertEqual([t.steps for t in pool._tasks], [0, 0])
        self.assertEqual([t.episodes for t in pool._tasks], [2, 2])
        self.assertTrue(np.array_equal(out["obs"][0], np.array(pool._tasks[0].observation(), dtype=np.float32)))


if __name__ == "__main__":
    unittest.main()
