"""Topological memory graph, trajectory selector, and closed-loop NoMaD navigator."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import nomad_cam, nomad_onnx

MAX_NODES = 64
PLAN_INTERVAL_TICKS = 50
EDGE_NEW_DIST_STEPS = 2.2
EDGE_NEW_METRIC_MM = 75.0
LOOP_CLOSE_DIST_STEPS = 3.0


def _clamp(value: float, low: float, high: float) -> float:
    assert low <= high, "invalid clamp bounds"
    return min(max(value, low), high)


@dataclass
class TopoNode:
    """One visual observation node in the topological navigation graph."""

    node_id: int
    image_u8: np.ndarray
    pose_mm: Tuple[float, float, float]
    label: str = ""
    visits: int = 1
    seen_tick: int = 0


@dataclass
class TopologicalGraph:
    """Visual topological memory graph with temporal-distance edges and bounded Dijkstra."""

    nodes: List[TopoNode] = field(default_factory=list)
    edges: Dict[int, Dict[int, float]] = field(default_factory=dict)
    current_id: Optional[int] = None

    def reset(self) -> None:
        self.nodes.clear()
        self.edges.clear()
        self.current_id = None

    def add_edge(self, src: int, dst: int, weight: float) -> None:
        assert 0 <= src < len(self.nodes) and 0 <= dst < len(self.nodes), "node index out of range"
        assert weight > 0.0, "edge weight must be positive"
        if src == dst:
            return
        cur_w = self.edges.setdefault(src, {}).get(dst, float("inf"))
        w = min(cur_w, float(weight))
        self.edges[src][dst] = w
        self.edges.setdefault(dst, {})[src] = w

    def _add_node(self, image_u8: np.ndarray, pose_mm: Tuple[float, float, float],
                  tick: int, label: str = "") -> int:
        nid = len(self.nodes)
        self.nodes.append(TopoNode(nid, np.array(image_u8, dtype=np.uint8, copy=True), pose_mm, label, 1, tick))
        self.edges[nid] = {}
        return nid

    def localize(self, obs_stack: np.ndarray, dist_fn: Callable[[np.ndarray, np.ndarray], np.ndarray],
                 pose_mm: Optional[Tuple[float, float]] = None) -> Tuple[Optional[int], np.ndarray]:
        """Return (closest_node_id, all_predicted_distances) across stored nodes."""
        if not self.nodes:
            return None, np.zeros((0,), dtype=np.float32)
        goal_batch = np.stack([nomad_onnx.rgb_to_chw(n.image_u8) for n in self.nodes], axis=0)
        obs_batch = np.repeat(obs_stack.reshape(1, 12, nomad_cam.IMG_H, nomad_cam.IMG_W), len(self.nodes), axis=0)
        dists = dist_fn(obs_batch, goal_batch).reshape(-1)
        scores = dists.copy()
        if pose_mm is not None:
            for idx, node in enumerate(self.nodes):
                scores[idx] += 0.008 * math.hypot(pose_mm[0] - node.pose_mm[0], pose_mm[1] - node.pose_mm[1])
        best_id = int(np.argmin(scores))
        return best_id, dists

    def record_step(self, obs_stack: np.ndarray, cur_img_u8: np.ndarray,
                    pose_mm: Tuple[float, float, float],
                    dist_fn: Callable[[np.ndarray, np.ndarray], np.ndarray],
                    tick: int = 0, label: str = "") -> int:
        """Localize against memory, add new node if novel, and link loop closures."""
        if not self.nodes:
            nid = self._add_node(cur_img_u8, pose_mm, tick, label)
            self.current_id = nid
            return nid
        closest_id, dists = self.localize(obs_stack, dist_fn, pose_mm[:2])
        assert closest_id is not None
        min_metric = min(math.hypot(pose_mm[0] - n.pose_mm[0], pose_mm[1] - n.pose_mm[1]) for n in self.nodes)
        cur_id = self.current_id if self.current_id is not None else closest_id
        if (float(dists[closest_id]) > EDGE_NEW_DIST_STEPS or min_metric > EDGE_NEW_METRIC_MM) and len(self.nodes) < MAX_NODES:
            nid = self._add_node(cur_img_u8, pose_mm, tick, label)
            self.add_edge(cur_id, nid, max(1.0, float(dists[cur_id])))
            for oid, d in enumerate(dists):
                od_mm = math.hypot(pose_mm[0] - self.nodes[oid].pose_mm[0], pose_mm[1] - self.nodes[oid].pose_mm[1])
                if oid != nid and float(d) <= LOOP_CLOSE_DIST_STEPS and od_mm < 190.0:
                    self.add_edge(oid, nid, max(1.0, float(d)))
            self.current_id = nid
            return nid
        self.nodes[closest_id].visits += 1
        self.nodes[closest_id].seen_tick = tick
        if cur_id != closest_id and float(dists[cur_id]) <= LOOP_CLOSE_DIST_STEPS:
            self.add_edge(cur_id, closest_id, max(1.0, float(dists[closest_id])))
        self.current_id = closest_id
        return closest_id

    def shortest_path(self, src: int, dst: int) -> List[int]:
        """Bounded O(V^2) Dijkstra path from src to dst node ids."""
        n = len(self.nodes)
        assert 0 <= src < n and 0 <= dst < n, "invalid topological node id"
        dist = [float("inf")] * n
        prev = [-1] * n
        visited = [False] * n
        dist[src] = 0.0
        for _ in range(n):
            u = -1
            best = float("inf")
            for idx in range(n):
                if not visited[idx] and dist[idx] < best:
                    best, u = dist[idx], idx
            if u < 0 or u == dst:
                break
            visited[u] = True
            for v_id, w in self.edges.get(u, {}).items():
                if not visited[v_id] and dist[u] + w < dist[v_id]:
                    dist[v_id], prev[v_id] = dist[u] + w, u
        if math.isinf(dist[dst]):
            return [src, dst] if src != dst else [src]
        rev: List[int] = []
        cur = dst
        for _ in range(n):
            rev.append(cur)
            if cur == src:
                break
            cur = prev[cur]
        return list(reversed(rev))

    def choose_subgoal(self, obs_stack: np.ndarray, target_id: int,
                       dist_fn: Callable[[np.ndarray, np.ndarray], np.ndarray],
                       pose_mm: Optional[Tuple[float, float]] = None) -> Tuple[int, np.ndarray, float]:
        """Pick next intermediate topological subgoal toward target_id."""
        assert 0 <= target_id < len(self.nodes), "target node id out of range"
        closest_id, dists = self.localize(obs_stack, dist_fn, pose_mm)
        assert closest_id is not None
        direct_dist = float(dists[target_id])
        if direct_dist <= 3.5:
            return target_id, self.nodes[target_id].image_u8, direct_dist
        path = self.shortest_path(closest_id, target_id)
        sub_id = path[1] if len(path) >= 2 else target_id
        return sub_id, self.nodes[sub_id].image_u8, direct_dist

    def snapshot(self) -> Dict[str, Any]:
        """Serialize nodes and undirected edges for visualization and logging."""
        seen = set()
        edge_list = []
        for u, nbrs in self.edges.items():
            for v, w in nbrs.items():
                pair = (min(u, v), max(u, v))
                if pair not in seen:
                    seen.add(pair)
                    edge_list.append({"u": pair[0], "v": pair[1], "weight": round(float(w), 2)})
        return {
            "nodes": [{"id": n.node_id, "x_mm": n.pose_mm[0], "y_mm": n.pose_mm[1],
                       "yaw_deg": n.pose_mm[2], "label": n.label, "visits": n.visits} for n in self.nodes],
            "edges": edge_list,
            "current_id": self.current_id,
        }


def select_safe_trajectory(cands_mm: np.ndarray, state: Any,
                           v: Dict[str, Any]) -> Tuple[np.ndarray, int]:
    """Select the highest-scoring trajectory candidate given cliff/ToF hazard state."""
    assert cands_mm.ndim == 3 and cands_mm.shape[1:] == (nomad_onnx.HORIZON, nomad_onnx.ACT_DIM), "bad candidates"
    best_idx = 0
    best_score = -float("inf")
    half_l = 0.5 * float(v["table_length_mm"]) - 65.0
    half_w = 0.5 * float(v["table_width_mm"]) - 65.0
    c_yaw, s_yaw = math.cos(state.pose_angle_rad), math.sin(state.pose_angle_rad)
    for idx in range(cands_mm.shape[0]):
        traj = cands_mm[idx]
        reach = float(math.hypot(float(traj[2, 0]), float(traj[2, 1])))
        end_x = float(state.x_mm) + c_yaw * float(traj[4, 0]) - s_yaw * float(traj[4, 1])
        end_y = float(state.y_mm) + s_yaw * float(traj[4, 0]) + c_yaw * float(traj[4, 1])
        score = 0.25 * float(traj[2, 0]) + 0.1 * reach
        if abs(end_x) > half_l or abs(end_y) > half_w:
            score -= 220.0
        if state.prox_found_object and float(state.prox_distance_mm) < 135.0:
            if float(traj[2, 0]) > 25.0 and abs(float(traj[2, 1])) < 24.0:
                score -= 180.0
        if state.cliff_detected and float(traj[1, 0]) > 0.0:
            score -= 250.0
        if score > best_score:
            best_score, best_idx = score, idx
    return cands_mm[best_idx], best_idx


def waypoint_to_wheels(traj_mm: Sequence[Sequence[float]], v: Dict[str, Any],
                       cliff_any: bool = False, prox_found: bool = False,
                       prox_mm: float = 400.0) -> Tuple[float, float]:
    """Track lookahead waypoint traj[1..2] as differential-drive left/right mm/s commands."""
    assert len(traj_mm) >= 3 and len(traj_mm[1]) == 2, "trajectory needs >= 3 waypoints"
    if cliff_any:
        return 0.0, 0.0
    tx = 0.5 * (float(traj_mm[1][0]) + float(traj_mm[2][0]))
    ty = 0.5 * (float(traj_mm[1][1]) + float(traj_mm[2][1]))
    dist = math.hypot(tx, ty)
    alpha = math.atan2(ty, tx)
    top = float(v["wheel_speed_max_mmps"])
    half = 0.5 * float(v["tread_spacing_mm"])
    if dist < 10.0:
        return 0.0, 0.0
    if abs(alpha) > 0.70:
        sign = 1.0 if alpha >= 0.0 else -1.0
        omega = sign * _clamp(abs(alpha) * 1.4, 0.8, 1.9)
        return _clamp(-omega * half, -top, top), _clamp(omega * half, -top, top)
    spd_cap = 105.0 if (not prox_found or prox_mm > 130.0) else 55.0
    if prox_found and prox_mm < 72.0:
        spd_cap = 0.0
    lin = _clamp(dist / 0.60, 22.0 if spd_cap > 0.0 else 0.0, spd_cap)
    omega = _clamp(2.2 * alpha, -1.6, 1.6)
    left = _clamp(lin - omega * half, -top, top)
    right = _clamp(lin + omega * half, -top, top)
    return left, right


class NoMaDNavigator:
    """Closed-loop NoMaD visual navigator + explorer driving VectorSim."""

    def __init__(self, sim: Any, policy: Any, seed: int = 0) -> None:
        self.sim = sim
        self.policy = policy
        self.graph = TopologicalGraph()
        self.frames_u8: List[np.ndarray] = []
        self.rng = np.random.default_rng(seed)
        self.goal_img_u8: Optional[np.ndarray] = None
        self.goal_node_id: Optional[int] = None
        self.last_pred_dist: float = 0.0
        self.last_candidates_mm: Optional[np.ndarray] = None
        self.last_selected_idx: int = 0
        self._wheel_cmd: Tuple[float, float] = (0.0, 0.0)
        self._recover_ticks = 0

    def _dist_batch(self, obs_batch: np.ndarray, goal_batch: np.ndarray) -> np.ndarray:
        mask = np.zeros((obs_batch.shape[0], 1), dtype=np.float32)
        _, pred_dist = self.policy.encode(obs_batch, goal_batch, mask)
        return pred_dist

    def _sample_camera_obs(self) -> np.ndarray:
        img = nomad_cam.render_head_cam_rgb(self.sim)
        self.frames_u8.append(img)
        if len(self.frames_u8) > nomad_onnx.CONTEXT_FRAMES:
            self.frames_u8.pop(0)
        return nomad_onnx.stack_obs_frames(self.frames_u8)

    def _pose_tuple(self) -> Tuple[float, float, float]:
        s = self.sim.state
        return (float(s.x_mm), float(s.y_mm), math.degrees(float(s.pose_angle_rad)))

    def capture_goal(self, label: str = "goal") -> int:
        """Record current head_cam view as a visual topological navigation goal."""
        obs_stack = self._sample_camera_obs()
        cur_img = self.frames_u8[-1]
        nid = self.graph.record_step(obs_stack, cur_img, self._pose_tuple(), self._dist_batch, self.sim.tick, label)
        self.goal_img_u8 = cur_img.copy()
        self.goal_node_id = nid
        return nid

    def _handle_safety_recovery(self) -> bool:
        if self.sim._react_phase != "idle":
            return True
        if not self.sim.has_control:
            self.sim.request_control()
            self._recover_ticks = 90
        if self._recover_ticks > 0:
            self._recover_ticks -= 1
            self.sim._wheel_target = [-65.0, -65.0] if self._recover_ticks > 45 else [-70.0, 70.0]
            return True
        return False

    def _plan_step(self, goal_img: Optional[np.ndarray], goal_mask: float,
                   goal_node_id: Optional[int]) -> None:
        obs_stack = self._sample_camera_obs()
        cur_img = self.frames_u8[-1]
        pose = self._pose_tuple()
        self.graph.record_step(obs_stack, cur_img, pose, self._dist_batch, self.sim.tick)
        if goal_mask >= 0.5 or goal_img is None:
            g_chw = nomad_onnx.rgb_to_chw(cur_img)[None, ...]
            mask_val = 1.0
        else:
            if goal_node_id is not None and 0 <= goal_node_id < len(self.graph.nodes):
                _, sub_img, direct_dist = self.graph.choose_subgoal(obs_stack, goal_node_id, self._dist_batch, pose[:2])
                g_chw = nomad_onnx.rgb_to_chw(sub_img)[None, ...]
                self.last_pred_dist = direct_dist
            else:
                g_chw = nomad_onnx.rgb_to_chw(goal_img)[None, ...]
            mask_val = 0.0
        ctx, pred_dist = self.policy.encode(obs_stack[None, ...], g_chw, np.array([[mask_val]], dtype=np.float32))
        if goal_mask < 0.5 and goal_node_id is None:
            self.last_pred_dist = float(pred_dist[0, 0])
        noise = self.rng.standard_normal((4, nomad_onnx.HORIZON, nomad_onnx.ACT_DIM)).astype(np.float32)
        cands = self.policy.ddim_sample(ctx[0], num_samples=4, init_noise=noise)
        self.last_candidates_mm = cands
        best_traj, best_idx = select_safe_trajectory(cands, self.sim.state, self.sim.v)
        self.last_selected_idx = best_idx
        self._wheel_cmd = waypoint_to_wheels(best_traj, self.sim.v, self.sim.hazards.cliff_any,
                                             self.sim.state.prox_found_object, self.sim.state.prox_distance_mm)

    def explore(self, duration_s: float = 10.0) -> Dict[str, Any]:
        """Run goal-masked exploration (mask=1), expanding TopologicalGraph safely."""
        assert 0.5 <= duration_s <= 60.0, "duration_s out of range"
        self.sim.command_head_angle(4.0)
        self.sim.command_lift_height(32.0)
        saved_in = self.sim._in_action
        self.sim._in_action = True
        try:
            total = int(round(duration_s / self.sim.dt))
            for t in range(total):
                if not self._handle_safety_recovery():
                    if t % PLAN_INTERVAL_TICKS == 0:
                        self._plan_step(None, 1.0, None)
                    self.sim._wheel_target = [self._wheel_cmd[0], self._wheel_cmd[1]]
                self.sim.step(1)
                self.sim._publish_state()
        finally:
            self.sim._in_action = saved_in
            self.sim._wheel_target = [0.0, 0.0]
        snap = self.graph.snapshot()
        return {"result": "completed", "nodes": len(snap["nodes"]), "edges": len(snap["edges"]),
                "reactions": self.sim.reactions, "fallen": bool(self.sim.sim_truth()["fallen"])}

    def navigate_to(self, goal_img_u8: Optional[np.ndarray] = None, max_s: float = 20.0,
                    goal_node_id: Optional[int] = None) -> Dict[str, Any]:
        """Visually navigate toward goal_img_u8 or goal_node_id via topological subgoals + DDIM."""
        assert 0.5 <= max_s <= 90.0, "max_s out of range"
        target_img = goal_img_u8 if goal_img_u8 is not None else self.goal_img_u8
        gid = goal_node_id if goal_node_id is not None else self.goal_node_id
        assert target_img is not None, "no target goal image captured"
        self.sim.command_head_angle(4.0)
        self.sim.command_lift_height(32.0)
        saved_in, close_streak, outcome = self.sim._in_action, 0, "timeout"
        self.sim._in_action = True
        try:
            for t in range(int(round(max_s / self.sim.dt))):
                if not self._handle_safety_recovery():
                    if t % PLAN_INTERVAL_TICKS == 0:
                        self._plan_step(target_img, 0.0, gid)
                        close_streak = close_streak + 1 if self.last_pred_dist <= 1.25 else 0
                        if close_streak >= 2:
                            outcome = "completed"
                            break
                    self.sim._wheel_target = [self._wheel_cmd[0], self._wheel_cmd[1]]
                self.sim.step(1)
                self.sim._publish_state()
        finally:
            self.sim._in_action = saved_in
            self.sim._wheel_target = [0.0, 0.0]
        return {"result": outcome, "pred_dist": round(self.last_pred_dist, 3),
                "nodes": len(self.graph.nodes), "edges": sum(len(v) for v in self.graph.edges.values()) // 2,
                "fallen": bool(self.sim.sim_truth()["fallen"])}
