"""Rank the assumed parameters by how much each one moves the training task..

    the GPU host python rank_assumed.py
    the GPU host python rank_assumed.py --episodes 24 --workers 3 --policy /root/vector-runs/smoke3/policy.onnx

Every entry of parameters.json whose provenance is "assumed" or "chosen" is
set to its low bound and then to its high bound, all else at the table's
values, and two things are measured against the unchanged table:

  closed loop  the training task itself (simulacrum.task, randomisation off)
               on the same seeds, driven by the hand-written no-slip
               controller and, with --policy, by an exported ONNX policy:
               change in reward per step, falls and cliff reactions.
  open loop    fixed wheel commands with sensor noise off: the mean change
               in true forward speed and turn rate in units of the reward's
               own widths, the largest change in any reported observation
               channel (the accelerometer apart, because contact
               spikes dominate it), and what happens at a table edge.

The order printed is by the largest change in reward per step. This measures
the model, not a robot: it says which unknowns the model is sensitive to,
not what their real values are. The last line is one JSON object.

What the order is worth. This is a screen, not a priority list. It moves one
value at a time to bounds the author chose, so it misses interactions and
its order depends on those bounds. The model is sensitive to small changes:
--null N reruns everything N times with the robot's mass changed by 0.1 to
N times 0.1 percent (a gram at most, less than a kitchen scale resolves),
and the largest change those runs show is the floor below which nothing
here means anything. A change of parts in a million is too gentle a null:
it showed a floor ten times lower than any real variant reaches. Entries
are marked when they clear the floor on reward, for either driver. A priority also needs what a measurement would cost and how much it
would narrow the range, which this script does not know.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import multiprocessing
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

from simulacrum import params  # noqa: E402

EXIT_OK, EXIT_USAGE = 0, 2
MAX_WORKERS, MAX_EPISODES, MAX_NULLS = 4, 256, 16
RANKED = ("assumed", "chosen")
NULL_NAME, NULL_STEP = "total_mass_g", 1.0e-3
NOISE_OFF = {"gyro_noise_rms_dps": 0.0, "accel_noise_rms_mg": 0.0, "tof_range_std_fraction": 0.0}
REPORTED = 13  # the first 13 observation channels are what the robot reports; the rest are command and action
# (name, start x in mm, start y in mm, ((left mm/s, right mm/s, seconds), ...)); all fit a 300 mm table.
OPEN_LOOP = (
    ("straight", -80.0, 0.0, ((100.0, 100.0, 1.5), (0.0, 0.0, 0.6), (-100.0, -100.0, 1.5), (0.0, 0.0, 0.6))),
    ("fast", -80.0, 0.0, ((200.0, 200.0, 0.6), (0.0, 0.0, 0.9), (-200.0, -200.0, 0.6), (0.0, 0.0, 0.9))),
    ("spin", 0.0, 0.0, ((-150.0, 150.0, 1.5), (0.0, 0.0, 0.6), (100.0, -100.0, 1.5), (0.0, 0.0, 0.6))),
    ("arc", -72.0, 0.0, ((120.0, 40.0, 0.9), (40.0, 120.0, 0.9), (0.0, 0.0, 0.6))),
    ("rest", 0.0, 0.0, ((0.0, 0.0, 1.2),)),
)
# (name, control level, wheel speed mm/s): a straight run that starts EDGE_START_MM short of the +x edge.
EDGES = (("override 100", "override", 100.0), ("default 100", "default", 100.0), ("default 200", "default", 200.0))
EDGE_START_MM, EDGE_SECONDS = 120.0, 3.0


def assumed_names(doc: Dict[str, Any]) -> List[str]:
    """Every entry the author set without a source: assumed about the robot, or chosen for the scene or solver."""
    names = sorted(name for name, entry in doc["parameters"].items()
                   if entry["provenance"] in RANKED and "low" in entry)
    assert len(names) > 0, "the table has no assumed entry"
    return names


def nudged(doc: Dict[str, Any], index: int) -> Dict[str, Any]:
    """The table with the robot's mass changed by a tenth of a percent per index: a change that should not matter."""
    assert 0 <= index < MAX_NULLS, "null index out of range"
    out = copy.deepcopy(doc)
    out["parameters"][NULL_NAME]["value"] = doc["parameters"][NULL_NAME]["value"] * (1.0 + NULL_STEP * (index + 1))
    assert out["parameters"][NULL_NAME]["value"] != doc["parameters"][NULL_NAME]["value"], "the nudge did nothing"
    return out


def variant(doc: Dict[str, Any], name: str, bound: str) -> Dict[str, Any]:
    """A copy of the table with one assumed entry moved to its low or high bound."""
    assert bound in ("low", "high"), "bound is low or high"
    entry = doc["parameters"][name]
    assert entry["provenance"] in RANKED and bound in entry, "only a ranged assumed or chosen entry can be varied"
    out = copy.deepcopy(doc)
    out["parameters"][name]["value"] = copy.deepcopy(entry[bound])
    assert out["parameters"][name]["value"] != entry["value"] or entry[bound] == entry["value"], "variant did not move"
    return out


def run_segments(values: Dict[str, Any], level: str, start: Tuple[float, float],
                 segments: Sequence[Sequence[float]], scales: Dict[str, float]) -> Dict[str, Any]:
    """Drive fixed wheel commands and return, per policy step, the true motion and the reported observation."""
    from simulacrum import task
    from simulacrum.robot import VectorSim
    sim = VectorSim(values=values, control_level=level, start_xy_mm=start)
    step_s = values["state_period_ms"] * 0.001
    ticks = int(round(values["state_period_ms"] / values["control_period_ms"]))
    motion: List[Tuple[float, float]] = []
    obs: List[Tuple[float, ...]] = []
    for left, right, seconds in segments:
        for _ in range(int(round(seconds / step_s))):
            x0, y0, _ = sim.pose_mm()
            yaw0 = sim.heading_total_rad()
            sim.set_wheel_motors(left, right)
            sim.step(ticks)
            x1, y1, _ = sim.pose_mm()
            yaw1 = sim.heading_total_rad()
            mid = 0.5 * (yaw0 + yaw1)
            forward = ((x1 - x0) * math.cos(mid) + (y1 - y0) * math.sin(mid)) / step_s
            motion.append((forward, math.degrees(yaw1 - yaw0) / step_s))
            obs.append(task.build_observation(sim.state, (0.0, 0.0), (0.0, 0.0), scales)[:REPORTED])
    assert len(motion) == len(obs) and len(obs) > 0, "no steps were run"
    return {"motion": motion, "obs": obs, "fallen": bool(sim.sim_truth()["fallen"]), "reactions": sim.reactions,
            "max_x_mm": max(state.x_mm for state in sim.states)}


def open_loop(doc: Dict[str, Any]) -> Dict[str, Any]:
    """Every open-loop scenario and edge run for one table, with sensor noise off."""
    from simulacrum import task
    values = params.values(doc, NOISE_OFF)
    scales = task.observation_scales(params.values(params.load()))
    runs = {name: run_segments(values, "default", (x, y), segments, scales) for name, x, y, segments in OPEN_LOOP}
    edges: Dict[str, Any] = {}
    edge = 0.5 * values["table_length_mm"]
    for name, level, speed in EDGES:
        out = run_segments(values, level, (edge - EDGE_START_MM, 0.0), ((speed, speed, EDGE_SECONDS),), scales)
        flagged = [i for i, row in enumerate(out["obs"]) if row[task.OBS_NAMES.index("cliff_detected")] > 0.5]
        edges[name] = {"fallen": out["fallen"], "reactions": out["reactions"],
                       "past_edge_mm": round(out["max_x_mm"] - edge, 1),
                       "cliff_step": flagged[0] if flagged else None}
    assert len(runs) == len(OPEN_LOOP) and len(edges) == len(EDGES), "a scenario was not run"
    return {"runs": runs, "edges": edges, "names": list(task.OBS_NAMES[:REPORTED])}


def open_loop_change(base: Dict[str, Any], other: Dict[str, Any], sigma_f: float, sigma_t: float) -> Dict[str, Any]:
    """How far one table's open-loop behaviour is from another's. Pure arithmetic on two open_loop results."""
    names = base["names"]
    assert sigma_f > 0.0 and sigma_t > 0.0 and len(names) == REPORTED, "bad widths or channel names"
    motion_sum, steps = 0.0, 0
    squares = [0.0] * REPORTED
    for name, _, _, _ in OPEN_LOOP:
        a, b = base["runs"][name], other["runs"][name]
        assert len(a["motion"]) == len(b["motion"]), "runs differ in length"
        for (fa, ta), (fb, tb), oa, ob in zip(a["motion"], b["motion"], a["obs"], b["obs"]):
            motion_sum += max(abs(fb - fa) / sigma_f, abs(tb - ta) / sigma_t)
            squares = [s + (y - x) ** 2 for s, x, y in zip(squares, oa, ob)]
            steps += 1
    rms = [math.sqrt(s / steps) for s in squares]
    accel = [i for i in range(REPORTED) if names[i].startswith("accel_")]
    worst = max((i for i in range(REPORTED) if i not in accel), key=lambda i: rms[i])
    assert len(accel) == 3, "the accelerometer has three channels"
    edge_notes = []
    for name, _, _ in EDGES:
        a, b = base["edges"][name], other["edges"][name]
        if (a["fallen"], a["reactions"]) != (b["fallen"], b["reactions"]) or a["cliff_step"] != b["cliff_step"] \
                or abs(a["past_edge_mm"] - b["past_edge_mm"]) >= 1.0:
            edge_notes.append(f"{name}: fell {a['fallen']}->{b['fallen']}, past edge {a['past_edge_mm']}->"
                              f"{b['past_edge_mm']} mm, flag step {a['cliff_step']}->{b['cliff_step']}")
    return {"motion": round(motion_sum / steps, 4), "obs_rms": round(rms[worst], 5),
            "obs_channel": names[worst], "accel_rms": round(max(rms[i] for i in accel), 5),
            "edge_flips": sum(
                1 for name, _, _ in EDGES if base["edges"][name]["fallen"] != other["edges"][name]["fallen"]),
            "edges": edge_notes, "steps": steps}


def closed_loop(doc: Dict[str, Any], episodes: int, seed: int, policy_path: Optional[str]) -> Dict[str, Any]:
    """The training task on fixed seeds with randomisation off, for the no-slip controller and an ONNX policy."""
    from simulacrum import task
    assert 1 <= episodes <= MAX_EPISODES, "episode count out of range"
    drivers: Dict[str, Any] = {"no_slip": None}
    if policy_path:
        from simulacrum import onnx_policy
        drivers["policy"] = onnx_policy.OnnxPolicy(Path(policy_path))
    out: Dict[str, Any] = {}
    for label, policy in drivers.items():
        reward, steps, ends = 0.0, 0, {task.END_FALL: 0, task.END_REACTION: 0, task.END_TIME: 0}
        for episode in range(episodes):
            env = task.VectorTask(seed + episode, task.TaskConfig(randomise=False), doc=doc)
            for _ in range(env.max_steps):
                action = policy.act(env.observation()) if policy else task.kinematic_action(env.command, env.nominal)
                result = env.step(action)
                reward, steps = reward + result.reward, steps + 1
                if result.terminated or result.truncated:
                    ends[result.end_reason] += 1
                    break
        assert sum(ends.values()) == episodes, "an episode did not end"
        out[label] = {"reward_per_step": reward / steps, "steps": steps, "falls": ends[task.END_FALL],
                      "reactions": ends[task.END_REACTION], "time_limits": ends[task.END_TIME]}
    return out


def measure(job: Tuple[str, str, int, int, Optional[str]]) -> Dict[str, Any]:
    """One worker's job: everything measured for one variant of the table ("", "" is the table itself)."""
    name, bound, episodes, seed, policy_path = job
    doc = params.load()
    table = doc
    if bound.startswith("null"):
        table = nudged(doc, int(bound[4:]))
    elif name:
        table = variant(doc, name, bound)
    out = {"name": name, "bound": bound, "open": open_loop(table),
           "closed": closed_loop(table, episodes, seed, policy_path)}
    assert set(out["closed"]) >= {"no_slip"}, "the no-slip controller was not run"
    return out


def floor(base: Dict[str, Any], nulls: Sequence[Dict[str, Any]], sigma_f: float, sigma_t: float) -> Dict[str, Any]:
    """The largest change a change that should not matter produced: per driver for reward, and for motion."""
    out: Dict[str, Any] = {"runs": len(nulls), "motion": 0.0, "reward": {k: 0.0 for k in base["closed"]}}
    for row in nulls:
        out["motion"] = max(out["motion"], open_loop_change(base["open"], row["open"], sigma_f, sigma_t)["motion"])
        for k in base["closed"]:
            change = abs(row["closed"][k]["reward_per_step"] - base["closed"][k]["reward_per_step"])
            out["reward"][k] = max(out["reward"][k], change)
    assert set(out["reward"]) == set(base["closed"]), "a driver has no floor"
    return out


def rank(base: Dict[str, Any], rows: Sequence[Dict[str, Any]], sigma_f: float, sigma_t: float,
         floors: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
    """One line per assumed entry, worst of its two bounds, ordered by the change in reward per step."""
    by_name: Dict[str, Dict[str, Any]] = {}
    for row in rows:
        change = open_loop_change(base["open"], row["open"], sigma_f, sigma_t)
        d_reward = {k: row["closed"][k]["reward_per_step"] - base["closed"][k]["reward_per_step"] for k in base["closed"]}
        d_falls = {k: row["closed"][k]["falls"] - base["closed"][k]["falls"] for k in base["closed"]}
        entry = by_name.setdefault(row["name"], {"name": row["name"], "reward": 0.0, "motion": 0.0, "obs_rms": 0.0,
                                                 "obs_channel": "", "accel_rms": 0.0, "edge_flips": 0, "detail": {},
                                                 "clears_floor": False})
        entry["reward"] = max(entry["reward"], max(abs(v) for v in d_reward.values()))
        if floors and floors["runs"] > 0:
            entry["clears_floor"] = entry["clears_floor"] or any(
                abs(v) > floors["reward"][k] for k, v in d_reward.items())
            entry["motion_clears_floor"] = entry.get("motion_clears_floor", False) or change["motion"] > floors["motion"]
        entry["motion"] = max(entry["motion"], change["motion"])
        entry["edge_flips"] = max(entry["edge_flips"], change["edge_flips"])
        entry["accel_rms"] = max(entry["accel_rms"], change["accel_rms"])
        if change["obs_rms"] >= entry["obs_rms"]:
            entry["obs_rms"], entry["obs_channel"] = change["obs_rms"], change["obs_channel"]
        entry["detail"][row["bound"]] = {"d_reward": d_reward, "d_falls": d_falls, "open": change}
    ordered = sorted(by_name.values(), key=lambda e: (-e["reward"], -e["motion"], -e["obs_rms"], e["name"]))
    assert len(ordered) == len(by_name) and all(len(e["detail"]) == 2 for e in ordered), "an entry lacks a bound"
    return ordered


def report(base: Dict[str, Any], ordered: Sequence[Dict[str, Any]], episodes: int, floors: Dict[str, Any]) -> None:
    assert len(ordered) > 0, "nothing was ranked"
    rewards = ", ".join(f"{k} {v:.4f}" for k, v in sorted(floors["reward"].items()))
    print(f"floor from {floors['runs']} runs with the mass changed by a gram at most: |d reward/step| {rewards}; "
          f"motion {floors['motion']:.3f}. 'reward' and 'motion' say whether either bound beats that floor.")
    for label, row in base["closed"].items():
        print(f"table as it stands, {label}: reward/step {row['reward_per_step']:.4f} over {row['steps']} steps in "
              f"{episodes} episodes; falls {row['falls']}, cliff reactions {row['reactions']}, time limits {row['time_limits']}")
    print("rank  parameter                        |d reward/step|  motion  obs rms (channel)            accel rms  edge flips  clears floor: reward  motion")
    for place, e in enumerate(ordered, start=1):
        print(f"{place:4d}  {e['name']:32s} {e['reward']:15.4f}  {e['motion']:6.3f}  "
              f"{e['obs_rms']:.5f} ({e['obs_channel']:17s})  {e['accel_rms']:9.5f}  {e['edge_flips']:d} of {len(EDGES)}      "
              f"{'yes' if e['clears_floor'] else 'no':>18s}  {'yes' if e.get('motion_clears_floor') else 'no':>6s}")
    for e in ordered:
        for bound in ("low", "high"):
            d = e["detail"][bound]
            rewards = ", ".join(f"{k} {v:+.4f} (falls {d['d_falls'][k]:+d})" for k, v in sorted(d["d_reward"].items()))
            print(f"  {e['name']} {bound}: {rewards}; motion {d['open']['motion']:.3f}; " + "; ".join(d["open"]["edges"]))


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="Rank the assumed parameters by their effect on the training task.")
    parser.add_argument("--episodes", type=int, default=24)
    parser.add_argument("--seed", type=int, default=5000)
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--policy", default=None, help="an exported policy.onnx to drive the closed loop as well")
    parser.add_argument("--only", action="append", default=[], help="rank only this assumed entry; may repeat")
    parser.add_argument("--null", type=int, default=6, help="runs with a change that should not matter, for the floor")
    args = parser.parse_args(argv)
    doc = params.load()
    names = args.only if args.only else assumed_names(doc)
    bad = [name for name in names if name not in assumed_names(doc)]
    if bad or not 1 <= args.workers <= MAX_WORKERS or not 1 <= args.episodes <= MAX_EPISODES \
            or not 0 <= args.null <= MAX_NULLS or (args.policy and not Path(args.policy).is_file()):
        print(f"error: workers 1..{MAX_WORKERS}, episodes 1..{MAX_EPISODES}, null 0..{MAX_NULLS}, --policy a file, "
              f"--only an assumed or chosen entry (not {bad})", file=sys.stderr)
        return EXIT_USAGE
    from simulacrum import task
    config = task.TaskConfig()
    jobs = [("", "", args.episodes, args.seed, args.policy)]
    jobs += [(NULL_NAME, f"null{i}", args.episodes, args.seed, args.policy) for i in range(args.null)]
    jobs += [(name, bound, args.episodes, args.seed, args.policy) for name in names for bound in ("low", "high")]
    with multiprocessing.get_context("spawn").Pool(args.workers) as pool:
        results = pool.map(measure, jobs, chunksize=1)
    base, nulls, rows = results[0], results[1:1 + args.null], results[1 + args.null:]
    floors = floor(base, nulls, config.sigma_forward_mmps, config.sigma_turn_dps)
    ordered = rank(base, rows, config.sigma_forward_mmps, config.sigma_turn_dps, floors)
    report(base, ordered, args.episodes, floors)
    keys = ("name", "reward", "motion", "obs_rms", "obs_channel", "accel_rms", "edge_flips", "clears_floor", "detail")
    for entry in ordered:
        entry.setdefault("motion_clears_floor", False)
    keys = keys + ("motion_clears_floor",)
    print("RANK " + json.dumps({"episodes": args.episodes, "seed": args.seed, "policy": bool(args.policy),
                                "base": base["closed"], "base_edges": base["open"]["edges"], "floor": floors,
                                "order": [{k: e[k] for k in keys} for e in ordered]}, sort_keys=True))
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
