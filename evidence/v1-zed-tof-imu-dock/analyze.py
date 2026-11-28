"""Reproduce unit 1 multi-distance ToF vs ZED 2 stereo, 2-heading IMU fit, and dock return."""

from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulacrum import imu_fit, params, prox

G_MMPS2 = 9806.65


def load_sdk_rows(path: Path) -> List[Dict[str, str]]:
    """Load non-empty SDK CSV rows."""
    assert path.is_file(), f"missing SDK CSV: {path}"
    with path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) > 100, f"too few SDK rows in {path}"
    return rows


def load_zed_rows(path: Path) -> List[Dict[str, Any]]:
    """Load valid ZED stereo pose frames."""
    assert path.is_file(), f"missing ZED jsonl: {path}"
    rows = [json.loads(line) for line in path.open("r", encoding="utf-8") if line.strip()]
    found = [r for r in rows if r.get("found")]
    assert len(found) > 100, f"too few found ZED frames in {path}"
    return found


def analyze_tof_steps(rows: Sequence[Dict[str, str]],
                      zed: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Pair docked and 5 off-dock stationary stops against ZED stereo range and canonical ProxSensor."""
    states = [r for r in rows if r["event"] == "state"]
    v1 = params.values(params.load(model="vector_1"))
    ps = prox.ProxSensor(v1, noise_seed=1)
    out: List[Dict[str, Any]] = []
    for phase in ("dock_pre", "tof_step_0", "tof_step_1", "tof_step_2", "tof_step_3", "tof_step_4"):
        sub = [r for r in states if r["phase"] == phase]
        assert len(sub) >= 10, f"missing phase {phase}"
        t_lo = float(sub[0]["wall_s"]) + (0.2 if phase == "dock_pre" else 0.8)
        t_hi = float(sub[-1]["wall_s"]) - 0.1
        win = [r for r in sub if t_lo <= float(r["wall_s"]) <= t_hi
               and abs(float(r["wl"])) < 1.0 and abs(float(r["wr"])) < 1.0] or sub
        uniq = [win[0]] + [win[i] for i in range(1, len(win))
                           if win[i]["prox_mm"] != win[i - 1]["prox_mm"]
                           or win[i]["prox_q"] != win[i - 1]["prox_q"]]
        up = np.array([float(r["prox_mm"]) for r in uniq])
        uq = np.array([float(r["prox_q"]) for r in uniq])
        z_win = [z for z in zed if t_lo <= z["wall"] <= t_hi]
        zx = float(np.median([z["x_m"] * 1000.0 for z in z_win]))
        zdx = float(np.median([z["dock_x_m"] * 1000.0 for z in z_win]))
        pitch_rad = float(np.mean([float(r["pitch_rad"]) for r in win]))
        # Use measured ZED distance minus nominal 8 mm chin offset as nominal target hit for ProxSensor flag check.
        rd = ps.read([max(30.0, zx - 8.0)] * len(ps.rays), float(np.mean([float(r["lift_mm"]) for r in win])), pitch_rad)
        out.append({
            "phase": phase, "n_states": len(win), "n_tof_uniq": len(uniq),
            "zed_x_mm": f"{zx:.1f}", "zed_dock_x_mm": f"{zdx:.1f}",
            "odo_x_mm": f"{float(np.median([float(r['x_mm']) for r in win])):.1f}",
            "pitch_deg": f"{math.degrees(pitch_rad):.2f}",
            "tof_mean_mm": f"{up.mean():.2f}",
            "tof_sd_mm": f"{up.std(ddof=1) if len(up) > 1 else 0.0:.2f}",
            "tof_sd_frac": f"{(up.std(ddof=1) / up.mean()) if len(up) > 1 else 0.0:.4f}",
            "tof_q_med": f"{np.median(uq):.4f}",
            "phys_found": f"{np.mean([int(r['found_obj']) for r in win]):.2f}",
            "sim_found": int(rd.found_object),
        })
    return out


def analyze_level_imu(rows: Sequence[Dict[str, str]]) -> List[Dict[str, Any]]:
    """Run simulacrum.imu_fit across the 12 settled level-table holds at 0 and 180 deg headings."""
    states = [r for r in rows if r["event"] == "state"]
    cmds = [r for r in rows if r["event"] == "command_head"]
    rets = [r for r in rows if r["event"] == "return_head"]
    assert len(cmds) == len(rets) == 12, "expected 12 level-table head holds"
    stretches: List[Any] = []
    meta: List[Dict[str, Any]] = []
    for i, (cmd, ret) in enumerate(zip(cmds, rets)):
        t_ret = float(ret["s"])
        t_next = float(cmds[i + 1]["s"]) if i + 1 < len(cmds) else t_ret + 3.0
        win = [r for r in states if t_ret + 0.8 <= float(r["s"]) <= min(t_ret + 2.3, t_next - 0.1)
               and abs(float(r["gy"])) < 0.03]
        assert len(win) >= 10, f"too few settled IMU samples for {cmd['detail']}"
        hd = math.radians(float(np.mean([float(r["head_deg"]) for r in win])))
        yaw = float(np.mean([float(r["yaw_rad"]) for r in win]))
        ax = float(np.mean([float(r["ax"]) for r in win]))
        ay = float(np.mean([float(r["ay"]) for r in win]))
        az = float(np.mean([float(r["az"]) for r in win]))
        stretches.append((hd, yaw, (ax, ay, az), len(win)))
        meta.append({"detail": cmd["detail"], "head_deg": math.degrees(hd), "yaw_deg": math.degrees(yaw),
                     "ax_mg": ax * 1000.0 / G_MMPS2, "ay_mg": ay * 1000.0 / G_MMPS2, "az_mg": az * 1000.0 / G_MMPS2})
    fit_nom = imu_fit.fit(stretches, free_scale=False)
    fit_kx = imu_fit.fit(stretches, free_scale=True)
    p_kx = _pack_params(fit_kx["values"], free_scale=True)
    rows_out: List[Dict[str, Any]] = []
    for st, info in zip(stretches, meta):
        pred = imu_fit.predicted(p_kx, st[0], st[1]) * 1000.0 / G_MMPS2
        rows_out.append({
            "hold": info["detail"], "head_deg": f"{info['head_deg']:.2f}", "yaw_deg": f"{info['yaw_deg']:.1f}",
            "ax_mg": f"{info['ax_mg']:.2f}", "ay_mg": f"{info['ay_mg']:.2f}", "az_mg": f"{info['az_mg']:.2f}",
            "res_x_mg": f"{pred[0] - info['ax_mg']:+.2f}",
            "res_y_mg": f"{pred[1] - info['ay_mg']:+.2f}",
            "res_z_mg": f"{pred[2] - info['az_mg']:+.2f}",
        })
    print("imu_fit nom rms_mg=", round(fit_nom["rms_mmps2"] * 1000.0 / G_MMPS2, 3),
          "pitch_deg=", round(math.degrees(fit_nom["values"]["pitch_rad"]), 3))
    print("imu_fit kx  rms_mg=", round(fit_kx["rms_mmps2"] * 1000.0 / G_MMPS2, 3),
          "pitch_deg=", round(math.degrees(fit_kx["values"]["pitch_rad"]), 3),
          "kx=", round(fit_kx["values"]["scale_x"], 5))
    return rows_out


def _pack_params(vals: Dict[str, float], free_scale: bool) -> List[float]:
    """Pack imu_fit value dict into vector order for imu_fit.predicted()."""
    base = [vals[k] for k in imu_fit.NAMES]
    if free_scale:
        base.append(vals[imu_fit.SCALE_NAME])
    return base


def summarize_undock_and_dock(rows: Sequence[Dict[str, str]],
                              zed: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Summarize ZED dock_x travel and SDK return-contact metrics for Visit 1."""
    states = [r for r in rows if r["event"] == "state"]
    u_rows = [r for r in states if r["phase"] == "undock"]
    zu = [z for z in zed if float(u_rows[0]["wall_s"]) <= z["wall"] <= float(u_rows[-1]["wall_s"])
          and z["source"] == "eyes"]
    d_rows = [r for r in states if r["phase"] == "dock_return"]
    first_on = next((i for i, r in enumerate(d_rows) if int(r["status"]) & 0x1000), None)
    assert first_on is not None and len(zu) >= 10, "missing undock ZED frames or charger contact"
    last_m5 = next(i for i in range(first_on, 0, -1)
                   if math.degrees(float(d_rows[i - 1]["pitch_rad"])) > -5.0
                   >= math.degrees(float(d_rows[i]["pitch_rad"])))
    sub_ret = d_rows[last_m5:first_on + 1]
    ws = [0.5 * (float(r["wl"]) + float(r["wr"])) for r in sub_ret]
    ts = [float(r["s"]) for r in sub_ret]
    trav = sum(0.5 * (ws[k] + ws[k + 1]) * (ts[k + 1] - ts[k]) for k in range(len(ws) - 1))
    w_fin = float(np.median([0.5 * (float(r["wl"]) + float(r["wr"])) for r in d_rows[max(0, first_on - 12):first_on]]))
    z_m5 = [z["dock_x_m"] * 1000.0 for z in zed if abs(z["wall"] - float(d_rows[last_m5]["wall_s"])) < 0.25]
    z_on = [z["dock_x_m"] * 1000.0 for z in zed if abs(z["wall"] - float(d_rows[first_on]["wall_s"])) < 0.25]
    return [{
        "undock_zed_start_mm": f"{zu[0]['dock_x_m'] * 1000.0:.1f}",
        "undock_zed_end_mm": f"{zu[-1]['dock_x_m'] * 1000.0:.1f}",
        "return_dt_m5_to_on_s": f"{float(d_rows[first_on]['s']) - float(d_rows[last_m5]['s']):.2f}",
        "return_odo_travel_mm": f"{trav:.1f}",
        "return_final_med_mmps": f"{w_fin:.1f}",
        "return_pitch_at_on_deg": f"{math.degrees(float(d_rows[first_on]['pitch_rad'])):.2f}",
        "return_zed_m5_mm": f"{statistics.median(z_m5):.1f}" if z_m5 else "nan",
        "return_zed_on_mm": f"{statistics.median(z_on):.1f}" if z_on else "nan",
    }]


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    """Write uniform list of dicts to CSV."""
    assert len(rows) > 0, f"empty output for {path}"
    cols = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=cols, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    """Run all Visit 1 ToF, level IMU, and ZED undock/dock analyses."""
    rows = load_sdk_rows(HERE / "unit1-visit1-tof-imu-dock.csv")
    zed = load_zed_rows(HERE / "visit1-zed.jsonl")
    tof_rows = analyze_tof_steps(rows, zed)
    imu_rows = analyze_level_imu(rows)
    dock_rows = summarize_undock_and_dock(rows, zed)
    write_csv(HERE / "tof-vs-zed.csv", tof_rows)
    write_csv(HERE / "level-imu-fit.csv", imu_rows)
    write_csv(HERE / "dock-return.csv", dock_rows)
    print(f"wrote {len(tof_rows)} ToF rows, {len(imu_rows)} IMU holds, {len(dock_rows)} dock summary")


if __name__ == "__main__":
    main()
