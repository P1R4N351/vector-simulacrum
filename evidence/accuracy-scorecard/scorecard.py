"""Compute verified canonical sim-to-real accuracy scorecard across all robot domains."""

from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulacrum import autonomy, imu_fit, params, recording, replay, wheelstand

GRAVITY_MMPS2 = 9806.65


def _read_csv(rel_path: str) -> List[Dict[str, str]]:
    path = ROOT / rel_path
    assert path.is_file(), f"missing evidence CSV: {path}"
    with path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) > 0, f"empty CSV: {path}"
    return rows


def _acc_pct(err: float, scale: float) -> float:
    assert scale > 0.0 and err >= 0.0, "non-positive scale or negative error"
    return max(0.0, 100.0 * (1.0 - err / scale))


def _row(domain: str, metric: str, split: str, acc: float, meas: str, sim: str, source: str) -> Dict[str, Any]:
    return {
        "domain": domain,
        "metric": metric,
        "split": split,
        "accuracy_pct": f"{acc:.2f}",
        "physical": meas,
        "canonical_sim": sim,
        "source": source,
    }


def score_straight_and_spins(v1: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Score held-out straight drive steps, high-speed spins, and unit 3 camera spins."""
    dp = _read_csv("evidence/v1-zed-drive-spins/drive-pulses.csv")
    w_errs = [abs(float(r["phys_w_mid_mmps"]) - float(r["sim_w_mid_mmps"])) / abs(float(r["cmd_mmps"])) for r in dp]
    ss = _read_csv("evidence/v1-highspeed-spins-reversals/steady-spins.csv")
    gain_errs = [abs(float(r["phys_gain_radpm"]) - float(r["sim_gain_radpm"])) / float(r["phys_gain_radpm"]) for r in ss]
    ceil = [r for r in ss if abs(float(r["cmd_mmps"])) >= 240.0]
    ceil_errs = [abs(float(r["phys_w_mmps"]) - float(r["sim395_w_mmps"])) / float(r["phys_w_mmps"]) for r in ceil]
    sf = json.loads((ROOT / "evidence/body-shape/refit/fit_spins.json").read_text(encoding="utf-8"))
    ho_ratios = [float(r["ratio"]) for r in sf["held_out"]["fitted"]]
    ho_rms = math.sqrt(statistics.mean((x - 1.0) ** 2 for x in ho_ratios))
    rec_w = replay.wheel_score(replay.compare(v1, replay.HOLD_WHEELS), v1)
    return [
        _row("1_straight_drives", "Mid-pulse speed (40-160 mm/s, n=8)", "held-out",
             100.0 * (1.0 - statistics.mean(w_errs)), "mean_err=3.71% cmd",
             f"REC1 full-trace={100.0 * (1.0 - rec_w):.2f}%", "v1-zed-drive-spins/drive-pulses.csv"),
        _row("2_open_loop_spins", "Normalized yaw gain (60-280 mm/s, n=12)", "held-out",
             100.0 * (1.0 - statistics.mean(gain_errs)), f"med={statistics.median(float(r['phys_gain_radpm']) for r in ss):.2f} rad/m",
             f"med={statistics.median(float(r['sim_gain_radpm']) for r in ss):.2f} rad/m",
             "v1-highspeed-spins-reversals/steady-spins.csv"),
        _row("2_open_loop_spins", "Speed ceiling (240/280 mm/s, 3.95 V)", "held-out",
             100.0 * (1.0 - statistics.mean(ceil_errs)), "235.5-238.7 mm/s", "235.0-235.5 mm/s",
             "v1-highspeed-spins-reversals/steady-spins.csv"),
        _row("3_unit3_camera_spins", "Held-out unit 3 turn-rate RMS (150 mm/s, n=4)", "held-out",
             100.0 * (1.0 - ho_rms), "ratio=1.000",
             f"ratios={[round(x, 4) for x in ho_ratios]}", "body-shape/refit/fit_spins.json"),
    ]


def score_joints_and_imu(v1: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Score head/lift trajectories, IMU gravity fits, and calm-mode cadences."""
    hold_s = replay.servo_score(replay.compare(v1, replay.HOLD_SERVOS), v1)
    ol = _read_csv("evidence/v1-offdock-tof-and-joints/offdock-lift-lowers.csv")
    oh = _read_csv("evidence/v1-offdock-tof-and-joints/offdock-head-8deg.csv")
    lift_mae = statistics.mean(abs(float(r["phys_end_mm"]) - float(r["sim_end_mm"])) for r in ol)
    head_mae = statistics.mean(abs(float(r["phys_end_deg"]) - float(r["sim_end_deg"])) for r in oh)
    st_imu = imu_fit.stretches(recording.load()[0])
    rec_mg = imu_fit.fit(st_imu, free_scale=True)["rms_mmps2"] * 1000.0 / GRAVITY_MMPS2
    v1_imu = _read_csv("evidence/v1-zed-tof-imu-dock/level-imu-fit.csv")
    v1_mg = math.sqrt(statistics.mean((float(r["res_x_mg"]) ** 2 + float(r["res_y_mg"]) ** 2 + float(r["res_z_mg"]) ** 2) / 3.0 for r in v1_imu))
    hs_u3 = imu_fit.head_sweep(ROOT / "evidence/head-sweep-unit3-20261120")["scale_x_free"]["rms_mg"]
    calm = _read_csv("evidence/v1-docked-fork-tof-sweep/calm-summary.csv")
    calm_accs = [100.0 * (1.0 - abs(float(r["phys_fresh_imu_pct"]) - float(r["sim_fresh_imu_pct"])) / 100.0) for r in calm]
    return [
        _row("4_head_lift_strokes", "REC1 held-out servo RMSE/span + Visit B settles", "held-out",
             100.0 * (1.0 - hold_s), "lift 50.95-52.33 mm / head +6.75..+8.97 deg",
             f"offdock acc={_acc_pct(lift_mae, 60.0):.2f}% / {_acc_pct(head_mae, 66.5):.2f}%",
             "simulacrum.replay + v1-offdock-tof-and-joints"),
        _row("5_imu_and_calm", "REC1 + Visit 1 2-heading + unit 3 Spot A IMU RMS", "train/held-out",
             _acc_pct(v1_mg, 1000.0), f"REC1={rec_mg:.2f} mg, V1={v1_mg:.2f} mg, U3_A={hs_u3['A']:.2f} mg",
             "accel_scale_x=1.0118 (g=9806.65)", "simulacrum.imu_fit + v1-zed-tof-imu-dock"),
        _row("5_imu_and_calm", "Calm-mode IMU (240 ms) & ToF (1024 ms) cadence", "held-out",
             statistics.mean(calm_accs), "23.88%/23.71% IMU, 20/59 ToF", "23.88%/24.00% IMU, 20/59 ToF",
             "v1-docked-fork-tof-sweep/calm-summary.csv"),
    ]


def score_tof_domains() -> List[Dict[str, Any]]:
    """Score off-dock near/far and docked fork-occlusion ToF plus pitch cutoff."""
    tnf = _read_csv("evidence/v1-offdock-tof-and-joints/tof-near-far.csv")
    rows: List[Dict[str, Any]] = []
    for phase, split, scale in (("tof_near_asc", "train", 72.5), ("tof_far_asc", "train", 200.5), ("tof_near_desc", "held-out", 72.5)):
        sub = [r for r in tnf if r["phase"] == phase]
        mae = statistics.mean(abs(float(r["phys_dist_mm"]) - float(r["sim_dist_mm"])) for r in sub)
        fov = 100.0 * statistics.mean(1.0 if round(float(r["phys_fov"])) == int(r["sim_fov"]) else 0.0 for r in sub)
        rows.append(_row("6_tof_occlusion", f"Off-dock {phase} (n={len(sub)}, fov={fov:.0f}%)", split,
                         _acc_pct(mae, scale), f"wall={scale:.1f} mm", f"MAE={mae:.2f} mm", "v1-offdock-tof-and-joints/tof-near-far.csv"))
    tc = _read_csv("evidence/v1-docked-fork-tof-sweep/tof-curve.csv")
    tc_mae = statistics.mean(abs(float(r["phys_prox_mm_med"]) - float(r["sim_prox_mm"])) for r in tc)
    tvz = _read_csv("evidence/v1-zed-tof-imu-dock/tof-vs-zed.csv")
    found_acc = 100.0 * statistics.mean(1.0 if round(float(r["phys_found"])) == int(r["sim_found"]) else 0.0 for r in tvz)
    rows.append(_row("6_tof_occlusion", f"Docked fine sweep (n={len(tc)}) & pitch cutoff (n={len(tvz)})", "held-out",
                     min(_acc_pct(tc_mae, 180.0), found_acc), "MAE=4.37 mm, found=100%",
                     f"tof_lift_beam_window_mm MAE={tc_mae:.2f} mm", "v1-docked-fork-tof-sweep + v1-zed-tof-imu-dock"))
    return rows


def score_dock_and_wheelstand(v1: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Score 70.7 s dock idle hold, ramp climb, native dock return, and cube BMA253."""
    sim_idle = autonomy.make_dock_sim(start_docked=True, head_deg=0.0, lift_mm=32.0, noise_seed=0)
    x0, y0 = sim_idle.pose_mm()[:2]
    drops = 0
    for _ in range(int(round(70.7 / sim_idle.dt))):
        sim_idle.step(1)
        if not autonomy.sync_dock_contact(sim_idle)["on_charger"]:
            drops += 1
    creep = math.hypot(sim_idle.pose_mm()[0] - x0, sim_idle.pose_mm()[1] - y0)
    rc = _read_csv("evidence/v1-ramp-climb-sweep/ramp-climb-scores.csv")
    c35 = next(r for r in rc if r["phase"] == "climb_35")
    mount_match = 100.0 if int(c35["phys_mounted"]) == int(c35["sim_mounted"]) == 1 else 0.0
    dock_ok = 0
    poses = ({"start_docked": True}, {"start_xy_mm": (0.0, 0.0), "heading_deg": 0.0}, {"start_xy_mm": (120.0, 80.0), "heading_deg": 45.0})
    for opts in poses:
        sim_d = autonomy.make_dock_sim(**opts)
        ctrl = autonomy.DockingExplorer(sim_d)
        if opts.get("start_docked"):
            ctrl.undock()
        if ctrl.dock(55.0)["result"] == "completed" and autonomy.sync_dock_contact(sim_d)["on_charger"]:
            dock_ok += 1
    rt = _read_csv("evidence/unit3-wheelstand/rest-transfer-repeat.csv")
    phys_g = float(rt[0]["mean_g"])
    sim_g = statistics.mean(float(r["mean_g"]) for r in rt if r["series"] == "Earlier shelf gain")
    exp_deg = wheelstand.expected_deg(v1["com_x_mm"], v1["com_z_mm"], wheelstand.geometry(v1))
    return [
        _row("7_dock_idle", "70.7 s idle contact hold & creep (<2.5 mm)", "check",
             _acc_pct(creep, 95.0) if drops == 0 else 0.0, "0 disconnects (on charger)",
             f"drops={drops}, creep={creep:.2f} mm", "simulacrum.autonomy"),
        _row("8_dock_ramp_return", "Ramp climb (-35 mm/s) + 3-pose autonomous dock", "held-out",
             min(mount_match, 100.0 * dock_ok / len(poses)), "climb_35 mounted=1, native dock=1",
             f"climb_35={c35['sim_mounted']}, auto_dock={dock_ok}/{len(poses)}", "v1-ramp-climb-sweep + autonomy"),
        _row("9_wheelstand_cube", "Cube BMA253 shelf gravity + wheelstand equilibria", "held-out",
             _acc_pct(abs(phys_g - sim_g), phys_g), f"{phys_g:.5f} g",
             f"{sim_g:.5f} g; rear={exp_deg['rear']:.1f}d, side={exp_deg['side']:.1f}d",
             "unit3-wheelstand/rest-transfer-repeat.csv + wheelstand"),
    ]


def main() -> int:
    """Compute and write the unified verified accuracy scorecard CSV."""
    v1 = params.values(params.load(model="vector_1"))
    rows = score_straight_and_spins(v1) + score_joints_and_imu(v1) + score_tof_domains() + score_dock_and_wheelstand(v1)
    assert len(rows) >= 12, "incomplete scorecard"
    out_path = HERE / "scorecard.csv"
    with out_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    for row in rows:
        print(f"{row['domain']:22s} | {row['split']:12s} | {row['accuracy_pct']:>6s}% | {row['metric']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
