"""Break each tested property once and show its test fails..

    the GPU host python tests/mutate.py        all arms (needs MuJoCo, torch, rsl_rl, onnxruntime)
    python3 tests/mutate.py                the arms that need none of those; the rest are NOT-RUN

For every arm: copy the skill to a temporary directory, swap one piece of
text (it must occur exactly once and the file must change), run the tests
named for that arm, and expect an assertion failure in one of them. Before
any arm, the same tests run on an untouched copy and must pass (the
control). One arm changes only a docstring and must leave the whole suite
green (the neutral change).

A fault in this script is reported as SETUP-ERROR, never as a verdict about
the tests. Exit codes: 0 every arm that ran was caught and the control and
neutral arms held; 7 an arm survived; 3 a set-up error.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))

import _common  # noqa: E402

RUN_TIMEOUT_S = 300
P, M, R, H = "simulacrum/params.py", "simulacrum/mjcf.py", "simulacrum/robot.py", "simulacrum/hazards.py"
T, V, TR, O, RP = ("simulacrum/task.py", "simulacrum/vec_env.py", "simulacrum/training.py",
                   "simulacrum/onnx_policy.py", "run_policy.py")
D, J = "simulacrum/drive.py", "parameters.json"
RC, RP2 = "simulacrum/recording.py", "simulacrum/replay.py"
HW, TRX = "hardware/record_barriered.py", "test_recorder."
IF, RK = "simulacrum/imu_fit.py", "rank_assumed.py"
MS, TMC = "simulacrum/mcp_service.py", "test_mcp."
WS, WC, TWS = "simulacrum/wheelstand.py", "wheelstand_check.py", "test_wheelstand."
PX, TPX = "simulacrum/prox.py", "test_prox."
SF, TSF = "simulacrum/spin_fit.py", "test_spin_fit.InsetFit."
CS, TCS = "simulacrum/camera_spins.py", "test_camera_spins.TurnCentres."
LP, TLP, TLR = "simulacrum/imu_filter.py", "test_imu_filter.Filter.", "test_imu_filter.InTheRobot."
RL, TRL = "hardware/record_listen.py", "test_record_listen.ListenOnly."
HC, THC = "hardware/hold_cube.py", "test_hold_cube.HoldCube."
CPS, TCPS = "hardware/capture_peers.py", "test_capture_peers.Capture."
SVC, TSVC = "hardware/save_capture.py", "test_capture_peers.Saver."
CA, TCP, TCC = "hardware/cube_accel.py", "test_cube_accel.Protocol.", "test_cube_accel.Collector."
CM, TCA, TCT, TCR = "simulacrum/cube.py", "test_cube_model.ChipAndApp.", "test_cube_model.Taps.", "test_cube_model.TurnCentre."
TCK, TCB = "test_cube_model.CarriedCube.", "test_cube_model.SpinWithTheCube."
SPIN_CUBE = TCB + "test_the_force_at_the_chip_gives_the_turn_centre_and_the_samples_agree_within_their_error"
TIF, TRK = "test_imu_fit.AccelerometerFit.", "test_rank."
TD, TRC, TRP = "test_drive.", "test_recording.RecordingFiles.", "test_replay.Replay."
TP, TM, TS, TT, TG = "test_params.", "test_mjcf.GeneratedModel.", "test_sim.", "test_task.", "test_training."
FALL_TEST = TT + "Episode.test_a_fall_ends_the_episode_as_a_failure_with_a_penalty"
VARIED_TEST = TT + "Randomisation.test_every_assumed_parameter_but_the_timestep_is_varied"
NO_TORCH_TEST = TG + "Pipeline.test_onnx_runs_without_torch_and_reproduces_the_recorded_closed_loop"

# (name, file, old text, new text, tests expected to fail, needs MuJoCo and the training packages)
ARMS: Tuple[Tuple[str, str, str, str, Tuple[str, ...], bool], ...] = (
    ("any provenance accepted", P, 'if entry.get("provenance") not in PROVENANCE:',
     'if entry.get("provenance") is None:',
     (TP + "ValidatorCatchesBadTables.test_unknown_provenance_is_rejected",), False),
    ("assumed value may lack a range", P,
     'needs = entry.get("provenance") in ("assumed", "fitted", "chosen")', "needs = False", (TP + "ValidatorCatchesBadTables.test_assumed_without_range_is_rejected",), False),
    ("value may sit outside its range", P, "if not numeric or not low <= val <= high:", "if not numeric:",
     (TP + "ValidatorCatchesBadTables.test_value_outside_its_range_is_rejected",), False),
    ("sampling ignores the seed", P, "rng = random.Random(seed)", "rng = random.Random()",
     (TP + "ValuesAndSampling.test_same_seed_same_sample_and_other_seed_differs",), False),
    ("wheel radius typed by hand", M,
     '    rf, rr = v["wheel_front_radius_mm"], v["wheel_rear_radius_mm"]\n    front, rear = wheel_x_mm(v)',
     '    rf, rr = 14.5, v["wheel_rear_radius_mm"]\n    front, rear = wheel_x_mm(v)',
     (TM + "test_wheel_radius_comes_from_the_table",), False),
    ("tread friction typed by hand", M, "f'{_contact_attrs(v, v[\"tread_friction\"])}/>", "f'{_contact_attrs(v, 0.9)}/>",
     (TM + "test_the_wheels_and_tip_supports_take_the_tread_friction",), False),
    ("table length typed by hand", M, 'half = (0.5 * v["table_length_mm"] * MM, 0.5 * v["table_width_mm"] * MM,',
     'half = (0.3, 0.5 * v["table_width_mm"] * MM,', (TM + "test_table_size_comes_from_the_table",), False),
    ("left and right front cliff sensors swapped", M, '("front_left", fx, y), ("front_right", fx, -y)',
     '("front_left", fx, -y), ("front_right", fx, y)',
     (TM + "test_cliff_sensors_sit_where_the_table_says_in_manual_order",), False),
    ("forward range sensor not tilted", M, "vec((math.cos(tilt), 0.0, math.sin(tilt)))", "vec((1.0, 0.0, 0.0))",
     (TM + "test_time_of_flight_site_is_tilted_up_by_the_table_angle",), True),
    ("centre of mass ignores the moving parts", M,
     'com_x = (total * v["com_x_mm"] - sum(p[0] * p[1] for p in points)) / masses["chassis"]',
     'com_x = v["com_x_mm"]', (TS + "ModelLoads.test_compiled_mass_and_centre_of_mass_match_the_table",), True),
    ("lift height drops the shoulder height", M,
     'return math.sin(angle_rad) * v["lift_arm_length_mm"] + v["lift_base_position_mm"][2]',
     'return math.sin(angle_rad) * v["lift_arm_length_mm"]',
     (TS + "Servos.test_reported_lift_height_is_the_height_of_the_wrist",), True),
    ("cliff reaction never starts", R, "if self.hazards.cliff_new and driving and not stopping:",
     "if False and self.hazards.cliff_new and driving and not stopping:",
     (TS + "Cliff.test_reaction_on_stops_backs_away_and_takes_control",), True),
    ("cliff reaction also runs at the override level", R,
     'if self.control_level != CONTROL_DEFAULT or self._react_phase != "idle":', 'if self._react_phase != "idle":',
     (TS + "Cliff.test_reaction_off_the_robot_drives_off_and_falls",), True),
    ("back-up distance halved", R, 'if gone >= v["cliff_reaction_backup_mm"] or self._react_ticks <= 0:',
     'if gone >= 0.5 * v["cliff_reaction_backup_mm"] or self._react_ticks <= 0:',
     (TS + "Cliff.test_reaction_on_stops_backs_away_and_takes_control",), True),
    ("reaction leaves control with the SDK", R, "self.has_control = False\n        self._cancelled = True",
     "self.has_control = True\n        self._cancelled = True",
     (TS + "Cliff.test_after_a_reaction_commands_are_refused_until_control_is_requested",), True),
    ("no pause before backing away", R,
     'self._react_ticks = int(round(v["cliff_reaction_pause_ms"] / v["control_period_ms"]))', "self._react_ticks = 0",
     (TS + "Cliff.test_control_cannot_be_taken_back_while_the_robot_is_still_reacting",), True),
    ("straight drive ends 10 mm early", R, 'if remaining <= v["drive_done_mm"]:',
     'if remaining <= 10.0 + v["drive_done_mm"]:',
     (TS + "Drive.test_slow_straight_drive_lands_on_the_distance",), True),
    ("turn direction reversed", R, "self._wheel_target = [-sign * w_cmd * half, sign * w_cmd * half]",
     "self._wheel_target = [sign * w_cmd * half, -sign * w_cmd * half]", (TS + "Drive.test_turn_left_quarter",), True),
    ("command delay ignored", R, "wheels = self._delay[(self._delay_at - self._wheel_delay) % size]",
     "wheels = slot",
     (TS + "Drive.test_command_takes_effect_after_the_command_delay",), True),
    ("state sent every control tick", R, "if self.tick % self._state_ticks == 0:", "if self.tick % 1 == 0:",
     (TS + "Drive.test_state_messages_come_at_the_state_rate",), True),
    ("wheel direction read from the model", R,
     'sign = math.copysign(1.0, truth[i]) if v["encoder_tread_direction_sensed"] else self._dir_sign[i]',
     "sign = math.copysign(1.0, truth[i])",
     (TS + "Drive.test_reported_wheel_speed_takes_its_sign_from_the_command",), True),
    ("reported wheel speed keeps its fraction", R,
     'out.append(whole_mmps(speed) if v["wheel_speed_reported_whole_mmps"] else speed)', "out.append(speed)",
     (TS + "Drive.test_wheel_speed_command_is_tracked_in_mm_per_s",), True),
    ("forward range has no noise", PX, 'raw *= 1.0 + self._noise.gauss(0.0, v["tof_range_std_fraction"])', "raw *= 1.0",
     (TS + "RangeSensors.test_forward_range_scatters_by_the_table_fraction",), True),
    ("forward range sees the robot's own lift", R, "scene_only[0] = 1  #", "scene_only[0] = scene_only[1] = 1  #",
     (TS + "RangeSensors.test_forward_range_reads_a_wall_that_fills_its_cone",), True),
    ("signal quality ignores how much of the cone is filled", PX, "quality = total / len(self.rays)",
     "quality = total / max(1, sum(1 for d in hits if d >= 0.0))",
     (TPX + "Signal.test_quality_falls_with_distance_and_with_how_much_of_the_cone_is_filled",), False),
    ("lift window ignored", PX, "        if in_beam:\n", "        if False:\n",
     (TPX + "Reporting.test_the_lift_is_in_the_beam_between_two_heights_and_lingers",), False),
    ("lift out of view at once", PX, "return self._clear_count < self._clear_needed", "return False",
     (TPX + "Reporting.test_the_lift_is_in_the_beam_between_two_heights_and_lingers",), False),
    ("pitch does not stop an object being found", PX, "and -limit <= pitch_rad <= limit)", "and True)",
     (TPX + "Reporting.test_nothing_is_found_while_the_robot_is_pitched",), False),
    ("reported distance not clamped at the top", PX,
     'distance = float(round(min(max(raw, v["tof_report_min_mm"]), v["tof_report_max_mm"])))',
     'distance = float(round(max(raw, v["tof_report_min_mm"])))',
     (TPX + "Reporting.test_the_distance_is_clamped_to_the_engines_limits",), False),
    ("weak reading reports no distance", PX, 'raw = self._noise.gauss(v["tof_weak_distance_mm"], v["tof_weak_distance_sd_mm"])',
     'raw = v["tof_report_max_mm"]', (TPX + "Signal.test_a_few_far_hits_are_too_weak_to_count",), False),
    ("gyro bias dropped", R, ' + math.radians(v["gyro_bias_dps"][i])', "",
     (TS + "RangeSensors.test_gyro_and_accelerometer_carry_the_table_noise_and_bias",), True),
    ("sensor noise ignores its seed", R, "self._noise = random.Random(noise_seed)", "self._noise = random.Random()",
     (TS + "Determinism.test_nominal_runs_repeat_exactly_including_a_fall",), True),
    ("head angle reported finer than one encoder count", R,
     "head = round(self.head_angle_rad() / head_step) * head_step", "head = self.head_angle_rad()",
     (TS + "Servos.test_head_reaches_its_limits_and_is_clamped",), True),
    ("encoder scale without the firmware's 0.96 factor", J, '"tread_mm_per_encoder_count": {"value": 0.126904,',
     '"tread_mm_per_encoder_count": {"value": 0.132192,', (TD + "Encoder.test_scale_is_the_firmware_line",), False),
    ("encoder never reads zero", D, "if self._idle_ticks >= self._zero_ticks:", "if False:",
     (TD + "Encoder.test_no_new_count_for_five_ticks_reads_zero",), False),
    ("encoder follows the direction of travel", D, "step = abs(travel_mm - self._last_travel)",
     "step = travel_mm - self._last_travel",
     (TD + "Encoder.test_one_channel_counts_backward_travel_as_travel",), False),
    ("feedforward line dropped from the speed loop", D,
     "out = self.feedforward(desired_mmps) + self._kp * error + self._ki * self.error_sum",
     "out = self._kp * error + self._ki * self.error_sum",
     (TD + "Controller.test_power_adds_proportional_and_integral_terms",), False),
    ("power applied against the commanded direction", D, "self.power = min(max(out, 0.0), self._max)",
     "self.power = min(max(out, -self._max), self._max)",
     (TD + "Controller.test_power_is_never_applied_against_the_commanded_direction",), False),
    ("a stop command keeps the motor powered", D, "if abs(desired_mmps) <= self._stopped:", "if False:",
     (TD + "Controller.test_a_stop_command_means_no_power_and_clears_the_integral",), False),
    ("tread motor has no friction", M, '"friction_Nm": v["tread_friction_power_fraction"] * stall}',
     '"friction_Nm": 0.0}', (TM + "test_tread_motor_is_the_linear_motor_the_table_describes",), False),
    ("stall torque typed by hand", M, 'stall = v["wheel_stall_torque_mNm"] * 0.001', "stall = 0.025",
     (TM + "test_every_ranged_robot_parameter_changes_the_model_or_is_used_by_the_layer",), False),
    ("state period back at the firmware's 30 ms", J, '"state_period_ms": {"value": 60.0,',
     '"state_period_ms": {"value": 30.0,', (TRC + "test_state_period_is_the_measured_interval",), False),
    ("gyro noise back at the datasheet's figure", J, '"gyro_noise_rms_dps": {"value": 0.034,',
     '"gyro_noise_rms_dps": {"value": 0.07,', (TRC + "test_noise_entries_are_the_measured_spread_at_rest",), False),
    ("cliff back-up back at the assumed 40 mm", J, '"cliff_reaction_backup_mm": {"value": 29.0,',
     '"cliff_reaction_backup_mm": {"value": 40.0,',
     (TRC + "test_cliff_reaction_entries_are_the_one_recorded_event",), False),
    ("state interval taken as the mean of every other row", RC,
     "gaps = [1000.0 * (b[\"t\"] - a[\"t\"]) for a, b in zip(states, states[1:])]",
     "gaps = [1000.0 * (b[\"t\"] - a[\"t\"]) for a, b in zip(states, states[2:])]",
     (TRC + "test_state_period_is_the_measured_interval",), False),
    ("a fit may be filed without citing the recording", P,
     'if prov in ("measured", "fitted") and "REC" not in text:', "if False:",
     (TP + "ValidatorCatchesBadTables.test_a_fit_may_not_be_filed_as_a_measurement_without_the_recording",), False),
    ("replay sends the wheel command 200 ms late", RP2, 'if tick == ticks_pre and seg.kind == "wheels":',
     'if tick == ticks_pre + 40 and seg.kind == "wheels":',
     (TRP + "test_wheel_segments_the_fit_used_match_the_robot",), True),
    ("replay never stops the treads", RP2, 'if tick == ticks_pre + ticks_on and seg.kind == "wheels":', "if False:",
     (TRP + "test_wheel_segments_held_out_match_the_robot_as_well",), True),
    ("replay compares the left record with the right tread", RP2,
     '_rmse([(m["wl"], r["wl"]) for m, r in zip(model, rows)]', '_rmse([(m["wr"], r["wl"]) for m, r in zip(model, rows)]',
     (TRP + "test_wheel_segments_the_fit_used_match_the_robot",), True),
    ("replay compares the gyro in the wrong unit", RP2, '(m["gyro_z_dps"], math.degrees(r["gyro"][2]))',
     '(m["gyro_z_dps"], r["gyro"][2])', (TRP + "test_wheel_segments_the_fit_used_match_the_robot",), True),
    ("head and lift commands act without their delay", R,
     "servos = self._delay[(self._delay_at - self._servo_delay) % size]", "servos = slot",
     (TRP + "test_head_and_lift_match_the_robot_on_fit_and_held_out_moves",), True),
    ("replay compares the left wheel's model speed with the right wheel's record", RP2,
     '+ [(m["wr"], r["wr"]) for m, r in zip(model, rows)])', '+ [(m["wl"], r["wr"]) for m, r in zip(model, rows)])',
     (TRP + "test_the_fast_spins_of_the_second_recording_replay",), True),
    ("set_wheel_motors clamped to the path-following limit", R, "        self._wheel_target = [left_mmps, right_mmps]",
     '        top = self.v["wheel_speed_max_mmps"]\n        self._wheel_target = [max(-top, min(top, left_mmps)), max(-top, min(top, right_mmps))]',
     (TS + "Drive.test_direct_drive_is_limited_by_the_motor_not_the_firmware_constant",), True),
    ("battery voltage does not reach the treads", R, "        scale = self.battery_volts / v[\"motor_reference_voltage_v\"]",
     "        scale = 1.0", (TS + "Drive.test_direct_drive_is_limited_by_the_motor_not_the_firmware_constant",), True),
    ("treads do not heat", R, "            self.tread_heat[i] += (abs(duties[i]) - self.tread_heat[i]) * min(1.0, self.dt / tau)\n",
     "            self.tread_heat[i] += 0.0\n",
     ("test_motor_heat.ModelUnderLoad.test_sustained_full_power_spins_slow_the_right_tread_as_fitted",), True),
    ("heat fit keeps spins a hazard ended", "simulacrum/motor_heat_fit.py",
     '        if stops[0].get("why") != "segment_end":\n            continue', '        if False:\n            continue',
     ("test_motor_heat.ConstructedRun.test_spins_ended_by_a_hazard_are_left_out",), False),
    ("vector 2 inherits measurements unmarked", "simulacrum/params.py",
     '            params[name] = dict(entry, inherited=variant["inherited_note"])', "            params[name] = entry",
     ("test_params.Vector2.test_it_replaces_only_what_it_names_and_marks_inherited_measurements",), False),
    ("vector 2 accepts overrides of unknown entries", "simulacrum/params.py",
     '        raise ValueError(f"variant overrides names the base table lacks: {unknown}")', "        pass",
     ("test_params.Vector2.test_a_variant_naming_an_unknown_entry_is_refused",), False),
    ("head and lift jump to full speed, no ramp", R,
     "    rate = min(cap, rate + accel * dt, math.sqrt(2.0 * accel * remaining))\n",
     "    rate = min(cap, math.sqrt(2.0 * accel * remaining))\n",
     (TRP + "test_head_and_lift_match_the_second_recording_which_no_fit_saw",), True),
    ("head starts away from where it was posed and travels there", R, "self._head_target = self._head_cmd = head",
     "self._head_target, self._head_cmd = head, 0.0",
     (TS + "Servos.test_head_and_lift_rates_change_nothing_while_neither_is_commanded",), True),
    ("same-way driving counted as the robot's fast turn", RC,
     'rows = [row for row in states if row["t"] > first and row["wl"] * row["wr"] < 0.0',
     'rows = [row for row in states if row["t"] > first and True',
     (TRC + "test_the_robots_own_fast_turn_stays_well_short_of_the_firmwares_spin_figure",), False),
    ("the table may describe other classes than the code allows", P,
     "if not isinstance(classes, dict) or sorted(classes) != sorted(PROVENANCE):", "if False:",
     (TP + "ValidatorCatchesBadTables.test_a_class_the_table_does_not_describe_is_rejected",), False),
    ("next recording: fast spins held 2 s, not 3", HW, 'wheels_for(robot, -speed, speed, 3.0, "spin_ccw_%d" % speed)',
     'wheels_for(robot, -speed, speed, 2.0, "spin_ccw_%d" % speed)',
     (TRX + "Sequence.test_default_phases_are_rest_then_head_and_lift_then_the_fast_spins",), False),
    ("next recording: the reversal stops between its halves", HW, "left2, right2, seconds2 = then",
     'left2, right2, seconds2 = then; stop(robot, "between")',
     (TRX + "Sequence.test_reversal_goes_straight_into_reverse_with_no_stop_between",), False),
    ("next recording: the repeated step is not the first recording's", HW,
     'wheels_for(robot, -80.0, -80.0, 0.3125, "repeat_back_80")', 'wheels_for(robot, -80.0, -80.0, 0.625, "repeat_back_80")',
     (TRX + "Sequence.test_repeat_is_the_spin_and_the_steps_of_the_first_recording",), False),
    ("next recording: a cliff no longer ends motion", HW, 'return "cliff_detected"', "return None",
     (TRX + "Edge.test_after_the_cliff_it_stops_then_only_listens_for_six_seconds",), False),
    ("next recording: the run ends at once after a hazard", HW, "        watch_after_abort()", "        pass",
     (TRX + "Edge.test_after_the_cliff_it_stops_then_only_listens_for_six_seconds",), False),
    ("next recording: the edge phase starts without the ruler distance", HW,
     "assert 5.0 <= start <= speed * seconds,", "assert start <= speed * seconds,",
     (TRX + "Edge.test_edge_phase_refuses_before_connecting_without_the_ruler_distance",), False),
    ("next recording: control read from the flag that never clears", HW, "    if granted is not None:", "    if False:",
     (TRX + "Safety.test_control_is_read_from_the_granted_event_not_the_flag_that_stays_set",), False),
    ("next recording: losing control is not a hazard", HW, 'return "control_lost"', "return None",
     (TRX + "Safety.test_losing_control_ends_the_run",), False),
    ("next recording: state rows drop the image time stamp", HW, ", img_ts=m.last_image_time_stamp)", ")",
     (TRX + "Rows.test_state_rows_carry_what_the_replay_reads_and_the_image_time",), False),
    ("reported distance not clamped at the bottom", PX,
     'distance = float(round(min(max(raw, v["tof_report_min_mm"]), v["tof_report_max_mm"])))',
     'distance = float(round(min(raw, v["tof_report_max_mm"])))',
     (TS + "RangeSensors.test_forward_range_never_reports_below_its_minimum",), True),
    ("start pose depends on the clock", R, "start_xy_mm[1] * mjcf.MM, 0.0",
     'start_xy_mm[1] * mjcf.MM, 1e-9 * (__import__("time").perf_counter_ns() % 997)',
     (TS + "Determinism.test_nominal_runs_repeat_exactly_including_a_fall",), True),
    ("falling is never reported", H, "if self._fall_ticks > self._fall_need:\n            self.falling = True",
     "if self._fall_ticks > self._fall_need:\n            self.falling = False",
     (TS + "Cliff.test_reaction_off_the_robot_drives_off_and_falls",), True),
    ("a drop within sensor reach is not a cliff", H, "self.cliff[i] = reading < 0.0 or reading > self._range_mm",
     "self.cliff[i] = reading < 0.0", (TS + "RangeSensors.test_front_cliff_sensors_over_the_edge_read_the_floor",), True),
    ("observation leaks the true position", T, 'state.prox_distance_mm / scales["prox"],',
     'state.x_mm / scales["prox"],', (TT + "Observation.test_observation_uses_only_what_the_robot_reports",), True),
    ("action not held to the wheel speed limit", T, "left = min(max(float(action[0]), -1.0), 1.0) * top",
     "left = float(action[0]) * top", (TT + "Action.test_action_is_two_wheel_speeds_inside_the_table_limit",), True),
    ("policy steps every control tick", T,
     'self.ticks = int(round(self.nominal["state_period_ms"] / self.nominal["control_period_ms"]))',
     "self.ticks = 1", (TT + "Action.test_policy_step_is_one_state_period",), True),
    ("domain randomisation switched off", T, "if not self.config.randomise:\n            return dict(self.nominal)",
     "if True:\n            return dict(self.nominal)", (VARIED_TEST,), True),
    ("one assumed parameter left out of the randomisation", T, 'NOT_RANDOMISED = ("sim_timestep_ms",)',
     'NOT_RANDOMISED = ("sim_timestep_ms", "tread_friction")', (VARIED_TEST,), True),
    ("a fall is treated as a time-out", T, "return True, False, END_FALL", "return False, True, END_FALL",
     (FALL_TEST,), True),
    ("no penalty for falling", T, "reward -= self.config.fall_penalty", "reward -= 0.0", (FALL_TEST,), True),
    ("a cliff reaction is blamed on the policy", T, "return False, True, END_REACTION",
     "return True, False, END_REACTION",
     (TT + "Episode.test_a_cliff_reaction_ends_the_episode_without_blaming_the_policy",), True),
    ("reward ignores the turn rate", T, "err_t = turn_dps - self.command[1]", "err_t = 0.0",
     (TT + "Episode.test_reward_is_highest_when_the_command_is_met",), True),
    ("pool ignores its seed", V, "self.seeds = [env_seed(seed, i) for i in range(num_envs)]",
     "self.seeds = [env_seed(0, i) for i in range(num_envs)]",
     (TT + "Pool.test_pool_seed_decides_the_environments",), True),
    ("more than four worker processes allowed", V, "MAX_WORKERS = 4", "MAX_WORKERS = 64",
     (TT + "Pool.test_more_than_four_worker_processes_is_refused",), True),
    ("a finished environment is not reset", V, "obs[i] = task.reset()", "obs[i] = result.observation",
     (TT + "Pool.test_a_finished_environment_is_reset_and_reported",), True),
    ("falls reported to PPO as time-outs", TR, '"time_outs": torch.from_numpy(out["truncated"].copy())',
     '"time_outs": torch.from_numpy(done.copy())', (TG + "Adapter.test_a_fall_is_not_reported_as_a_time_out",), True),
    ("the torch-free runner always reports success", RP, "return EXIT_OK if ok else EXIT_RED", "return EXIT_OK",
     (TG + "Pipeline.test_parity_check_fails_for_a_different_policy",), True),
    ("the torch-free runner imports torch", O, "import numpy as np\nimport onnxruntime\n",
     "import numpy as np\nimport onnxruntime\nimport torch\n", (NO_TORCH_TEST,), True),
    ("the recorded torch actions are not the policy's", TR,
     "action = deterministic_actions(runner, obs)[0]\n        obs_rows.append(obs[0])",
     "action = deterministic_actions(runner, obs)[0] * 0.5\n        obs_rows.append(obs[0])", (NO_TORCH_TEST,), True),
    ("a policy for another observation layout is accepted", O,
     'if tuple(self.meta["obs_names"]) != task.OBS_NAMES:', "if False:",
     (TG + "Pipeline.test_a_policy_for_another_observation_layout_is_refused",), True),
    ("a turn read from gyro z alone", RC, "rate = math.degrees(math.copysign(math.hypot(gx, gz), gz))",
     "rate = math.degrees(gz)", (TRC + "test_a_turn_is_read_from_gyro_x_and_z_together",), False),
    ("spin fraction taken over the whole segment, ramp included", RC,
     'rows = [r for r in states if 0.5 * (seg.t_cmd + seg.t_stop) <= r["t"] <= seg.t_stop]\n        ideal = ',
     'rows = [r for r in states if seg.t_cmd <= r["t"] <= seg.t_stop]\n        ideal = ',
     (TRC + "test_spin_fraction_entry_is_the_mean_of_the_six_recorded_spins",), False),
    ("still stretches keep their first and last rows", RC, "runs.append(current[trim:len(current) - trim])",
     "runs.append(current)", (TRC + "test_gyro_offset_entry_is_the_mean_reading_while_still",), False),
    ("gyro offset typed by hand", J, '"gyro_bias_dps": {"value": [0.001, 0.009, -0.001],',
     '"gyro_bias_dps": {"value": [0.001, 0.02, -0.001],',
     (TRC + "test_gyro_offset_entry_is_the_mean_reading_while_still",), False),
    ("spin fraction entry is the old gyro-z figure", J, '"spin_rate_fraction_measured": {"value": 0.5,',
     '"spin_rate_fraction_measured": {"value": 0.46,',
     (TRC + "test_spin_fraction_entry_is_the_mean_of_the_six_recorded_spins",), False),
    ("accelerometer fit: head pitch with the wrong sign", IF, "angle = head + p[2]", "angle = -head + p[2]",
     (TIF + "test_a_head_tilted_up_reads_gravity_on_plus_x",), False),
    ("accelerometer fit takes in the stretches on the charger", IF, 'if run[0]["t"] <= after_s:', "if False:",
     (TIF + "test_table_entries_are_the_fitted_values",), False),
    ("accelerometer fit stops after one step", IF, "MAX_ITERATIONS = 60", "MAX_ITERATIONS = 1",
     (TIF + "test_the_fit_recovers_made_up_offsets_at_the_recorded_head_angles_and_headings",), False),
    ("accelerometer x offset typed by hand", J, '"accel_bias_mg": {"value": [-47.0, -7.7, 22.3],',
     '"accel_bias_mg": {"value": [-40.0, -7.7, 22.3],', (TIF + "test_table_entries_are_the_fitted_values",), False),
    ("the robot's mass may be filed as chosen", P,
     'if (prov == "chosen") != (entry.get("kind") in CHOSEN_KINDS):', "if False:",
     (TP + "ValidatorCatchesBadTables.test_only_scene_and_solver_settings_may_be_filed_as_chosen",), False),
    ("ranking: a variant moves nothing", RK, 'out["parameters"][name]["value"] = copy.deepcopy(entry[bound])',
     'out["parameters"][name]["value"] = copy.deepcopy(entry["value"])',
     (TRK + "Arithmetic.test_a_variant_moves_one_assumed_entry_to_its_bound_and_nothing_else",), False),
    ("ranking: turn-rate change left out of the motion figure", RK,
     "motion_sum += max(abs(fb - fa) / sigma_f, abs(tb - ta) / sigma_t)", "motion_sum += abs(fb - fa) / sigma_f",
     (TRK + "Arithmetic.test_change_is_measured_in_the_rewards_own_widths",), False),
    ("ranking: order by name, not by reward change", RK,
     'key=lambda e: (-e["reward"], -e["motion"], -e["obs_rms"], e["name"])', 'key=lambda e: e["name"]',
     (TRK + "Arithmetic.test_order_is_by_the_larger_reward_change_of_the_two_bounds",), False),
    ("ranking: an entry clears the floor whatever it did", RK,
     'entry["clears_floor"] = entry["clears_floor"] or any(', 'entry["clears_floor"] = True or any(',
     (TRK + "Arithmetic.test_an_entry_clears_the_floor_only_by_beating_the_null_runs",), False),
    ("IMU pitch applied with the wrong sign", M, "imu_quat = (math.cos(half), 0.0, -math.sin(half), 0.0)",
     "imu_quat = (math.cos(half), 0.0, math.sin(half), 0.0)",
     (TS + "RangeSensors.test_accelerometer_at_rest_reads_gravity_in_the_heads_pitched_frame_plus_the_offsets",), True),
    ("accelerometer scale dropped", R, ' * v["accel_scale"][i] + mg', " + mg",
     (TS + "RangeSensors.test_the_x_scale_multiplies_the_reading_of_gravity_before_the_offset",), True),
    ("accelerometer scale not undone in the tilt", WS, ') / v["accel_scale"][i] for i in range(3))', ') for i in range(3))',
     (TWS + "Tilt.test_an_x_axis_scale_unlike_the_others_is_undone_before_the_tilt",), False),
    ("accelerometer fit ignores its x scale", IF, "kx * g * (ux * math.cos(angle)", "g * (ux * math.cos(angle)",
     ("test_imu_fit.XScaleOnUnit3.test_a_made_up_scale_is_recovered_at_the_recorded_head_angles",
      "test_imu_fit.XScaleOnUnit3.test_freeing_the_x_scale_lowers_the_error_at_the_spot_the_fit_never_saw"), False),
    ("accelerometer fit runs with a number that moves nothing", IF,
     '    assert int(np.linalg.matrix_rank(jac)) == len(names), "a fitted number has no effect on the readings"\n', "",
     ("test_imu_fit.XScaleOnUnit3.test_a_fit_where_some_number_cannot_move_the_readings_is_refused",), False),
    ("tip supports left out", M, "        + _tip_supports_xml(v)\n", "",
     ("test_wheelstand.Physics.test_the_model_balances_where_the_closed_form_says",), True),
    ("contact inset ignored", M, "    return -inset, -(base - inset)\n", "    return 0.0, -base\n",
     (TM + "test_the_contact_inset_moves_the_wheels_inward_and_keeps_the_body_level",), False),
    ("spin fit compares robot over model", SF, '"ratio": model["rate"] / robot["rate"]', '"ratio": robot["rate"] / model["rate"]',
     (TSF + "test_with_no_inset_the_model_spins_too_slowly_and_at_the_fit_it_does_not",), True),
    ("spin roll keeps the spin's share", SF, "return gyro[0] - gyro[2] * math.tan(head_rad), gyro[1]",
     "return gyro[0], gyro[1]", (TSF + "test_roll_from_the_head_gyro_leaves_out_the_spins_share",), True),
    ("spin fit takes the grid point below the crossing", SF,
     "return grid[k] + (1.0 - lo) * (grid[k + 1] - grid[k]) / (hi - lo)", "return grid[k]",
     (TSF + "test_the_crossing_is_interpolated_and_a_grid_that_never_reaches_one_gives_none",
      TSF + "test_the_table_holds_the_fitted_inset_and_its_range_covers_each_spins_fit"), True),
    ("held-out spins get the fitted inset both times", SF, "dict(values, tread_contact_inset_mm=0.0)",
     "dict(values, tread_contact_inset_mm=inset)", (TSF + "test_held_out_unit3_spins_come_closer_to_the_robots_rate",), True),
    ("contact inset typed by hand", J, '"tread_contact_inset_mm": {"value": 3.1,', '"tread_contact_inset_mm": {"value": 3.5,',
     (TSF + "test_the_table_holds_the_fitted_inset_and_its_range_covers_each_spins_fit",), True),
    ("camera turn centre read behind the lens", CS, '"centre_x_mm": LENS_X_MM + float(np.median(fwd))',
     '"centre_x_mm": LENS_X_MM - float(np.median(fwd))',
     (TCS + "test_the_four_spins_turn_about_the_points_the_readme_gives",
      TCS + "test_constructed_spins_return_the_turn_centres_they_were_built_with"), True),
    ("camera lens picks put on the shelf, not at lens height", CS,
     'lens = intersect(np.array([[float(r["u_px"]), float(r["v_px"])] for r in rows]), height, pose)',
     'lens = intersect(np.array([[float(r["u_px"]), float(r["v_px"])] for r in rows]), 0.0, pose)',
     (TCS + "test_constructed_spins_return_the_turn_centres_they_were_built_with",), True),
    ("model turn centre read behind the lens", SF, "return LENS_X_MM + float(np.median(ahead))",
     "return LENS_X_MM - float(np.median(ahead))",
     (TSF + "test_the_inset_keeps_the_turn_centre_within_5_mm_of_where_unit3_turns",), True),
    ("cube hold asks for default control", HC, "ControlPriorityLevel.RESERVE_CONTROL", "ControlPriorityLevel.DEFAULT_PRIORITY",
     (THC + "test_reserve_control_renews_inside_the_expiry_reads_batteries_and_lets_the_cube_go",), False),
    ("cube hold ignores a low robot battery", HC, 'if STATE["volts"] is not None and STATE["volts"] < LOW_V:',
     'if False:', (THC + "test_a_low_robot_battery_ends_the_hold_and_lets_the_cube_go",), False),
    ("cube hold renews only once", HC, "next_renew = now + RENEW_S", "next_renew = now + HOLD_S",
     (THC + "test_reserve_control_renews_inside_the_expiry_reads_batteries_and_lets_the_cube_go",
      THC + "test_a_failed_renewal_is_recorded_and_the_hold_goes_on"), False),
    ("cube hold never lets the cube go", HC, "                robot.world.disconnect_cube()\n", "                pass\n",
     (THC + "test_reserve_control_renews_inside_the_expiry_reads_batteries_and_lets_the_cube_go",
      THC + "test_a_stop_signal_ends_the_hold_early_and_still_releases_the_cube"), False),
    ("cube collector reads the samples one byte early", CA, 'struct.unpack_from("<9h", data, 2)',
     'struct.unpack_from("<9h", data, 1)', (TCP + "test_an_accelerometer_message_decodes_as_protocol_h_lays_it_out",),
     False),
    ("cube collector halves the scale", CA, "COUNTS_PER_G = 0x2000", "COUNTS_PER_G = 0x1000",
     (TCP + "test_an_accelerometer_message_decodes_as_protocol_h_lays_it_out",), False),
    ("cube app written from byte 0", CA, "range(APP_OFFSET, len(app), CHUNK)", "range(0, len(app), CHUNK)",
     (TCP + "test_an_app_file_is_written_from_byte_16_in_writes_of_at_most_20_bytes",
      TCC + "test_a_version_mismatch_uploads_the_app_from_byte_16_with_response_then_collects"), False),
    ("cube app written without response", CA, "chunk, response=True)", "chunk, response=False)",
     (TCC + "test_a_version_mismatch_uploads_the_app_from_byte_16_with_response_then_collects",), False),
    ("cube collector connects with no power named", CA, 'POWER in ("supply", "rechargeable")',
     'POWER in ("supply", "rechargeable", "")',
     (TCC + "test_collect_is_refused_without_a_supply_or_rechargeable_before_any_radio_use",), False),
    ("cube collector uploads over a matching app", CA, "    if version == expected:\n        return True\n",
     "    if False:\n        return True\n", (TCC + "test_a_matching_version_collects_messages_and_writes_nothing",), False),
    ("cube collector trusts any app file", CA, "if digest != DFU_SHA256:", "if False:",
     (TCC + "test_an_app_file_with_another_hash_is_refused_before_the_radio_is_used",), False),
    ("cube collector picks the first of several cubes", CA, "return cubes[0] if len(cubes) == 1 else None",
     "return cubes[0] if cubes else None",
     (TCC + "test_several_cubes_and_no_address_means_no_connection_but_an_address_picks_one",), False),
    ("cube collector ignores a low cube battery", CA, 'if STATE["low"] >= LOW_READINGS:', "if False:",
     (TCC + "test_three_low_battery_readings_end_the_run",), False),
    ("cube tick keeps the first frame it reads", CM, "self.slots[self.slot] = list(frames[-1])",
     "self.slots[self.slot] = list(frames[0])",
     (TCA + "test_each_tick_keeps_the_last_frame_it_reads_and_a_frame_due_with_the_tick_is_read",), False),
    ("cube frame due with a tick waits for the next one", CM, "if frame_at <= tick_at + TIME_EPS_S:",
     "if frame_at < tick_at - TIME_EPS_S:",
     (TCA + "test_each_tick_keeps_the_last_frame_it_reads_and_a_frame_due_with_the_tick_is_read",), False),
    ("the runner lets any skip pass under --require-mujoco", "tests/run_all.py",
     "require_mujoco and len(result.skipped) > len(optional)", "require_mujoco and False",
     ("test_run_all.Verdict.test_any_other_skip_fails_under_require_mujoco_and_passes_without_it",), False),
    ("cube mean does not round", CM, "return (16 * total + len(values) // 2) // len(values)",
     "return (16 * total) // len(values)",
     ("test_cube_firmware.TheModelsMean.test_the_mean_drops_the_low_bits_and_rounds_halves_up",), False),
    ("cube FIFO never fills", CM, "if len(self.fifo) >= FIFO_FRAMES:", "if False:",
     (TCA + "test_a_late_tick_finds_a_full_fifo_and_keeps_an_old_frame",), False),
    ("cube chip has thirteen bits", CM, "DATA_BITS = 12 ", "DATA_BITS = 13 ",
     (TCA + "test_values_are_twelve_bits_left_justified_and_clipped_at_four_g",), False),
    ("cube noise ignores the step length", CM, "math.sqrt(0.5 / step_s)", "math.sqrt(0.5)",
     (TCA + "test_noise_comes_out_at_the_density_through_the_filter",), False),
    ("cube reading keeps the four low bits", CM, "((int(c) >> LEFT_SHIFT) << LEFT_SHIFT)", "int(c)",
     (TCA + "test_reading_drops_the_four_low_bits_a_real_chip_may_fill",), False),
    ("cube taps need a second edge after a drop too", CM, "limit = -TAP_THRESH if self.pos_first else TAP_DEBOUNCE",
     "limit = -TAP_THRESH", (TCT + "test_as_shipped_a_sharp_drop_counts_at_the_next_frame_that_does_not_rise_by_90",),
     False),
    ("cube tap window never closes", CM, "if self.debounce > TAP_DEBOUNCE - TAP_DURATION and diff < limit:",
     "if diff < limit:", (TCT + "test_a_rise_that_falls_back_too_late_or_too_little_is_not",), False),
    ("cube turn centre divides by omega, not its square", CM, "(omega_radps * omega_radps)", "abs(omega_radps)",
     (TCR + "test_the_centre_comes_back_from_samples_built_for_it",
      TCR + "test_a_chip_carried_round_a_centre_reports_it_through_the_whole_chain"), False),
    ("carried cube fed in m/s^2, not g", R, "dtype=float) / g\n", "dtype=float)\n",
     (TCK + "test_held_still_at_carry_height_the_chip_reads_one_g_up", SPIN_CUBE), True),
    ("cube carried at the wrist itself", M, 'pos="{vec([c * MM for c in v["cube_carry_offset_mm"]])}"', 'pos="0 0 0"',
     (TCK + "test_the_model_carries_the_cube_where_the_engine_puts_it", SPIN_CUBE), True),
    ("listen-only recorder asks for control", RL, "behavior_control_level=None", 'behavior_control_level="default"',
     (TRL + "test_it_asks_for_no_control_records_for_the_time_given_and_commands_nothing",), False),
    ("listen-only recorder never puts quiet mode back", RL, '        if restore:\n            quiet("POST", enable=True)\n',
     "        pass\n", (TRL + "test_quiet_mode_on_is_lifted_then_put_back",
                        TRL + "test_quiet_mode_is_put_back_and_the_lock_released_when_the_connection_fails"), False),
    ("listen-only recorder lifts quiet mode that was off", RL, 'if before is not None and before.get("enabled"):',
     "if before is not None:", (TRL + "test_quiet_mode_off_is_left_alone",), False),
    ("IMU filter corners ignore the second section", LP,
     "x = 2.0 / ((1.0 + q2) + math.sqrt((1.0 + q2) ** 2 + 4.0 * q2))", "x = 1.0",
     (TLP + "test_the_two_sections_together_are_minus_3_db_at_the_cutoff",
      TLP + "test_the_discrete_filter_is_minus_3_db_within_one_percent_at_either_timestep"), False),
    ("IMU filter runs one section", LP, "self.state[1] += self.gains[1] * (self.state[0] - self.state[1])",
     "self.state[1] = self.state[0]", (TLP + "test_a_100_hz_tone_is_cut_as_one_or_two_sections_cut_it",), False),
    ("IMU filter corner taken as rad/s", LP, "1.0 - math.exp(-2.0 * math.pi * corner * step_s)",
     "1.0 - math.exp(-corner * step_s)",
     (TLP + "test_the_discrete_filter_is_minus_3_db_within_one_percent_at_either_timestep",), False),
    ("gyro cutoff typed by hand", J, '"gyro_filter_cutoff_hz": {"value": 18.65,', '"gyro_filter_cutoff_hz": {"value": 18.0,',
     (TLP + "test_the_table_holds_the_firmwares_settings_as_the_datasheet_rates_them",), False),
    ("robot reads the raw sensor, not the filter", R,
     "v, gauss, out = self.v, self._noise.gauss, self._imu_filter.output",
     "v, gauss, out = self.v, self._noise.gauss, self.data.sensordata[self._imu_adr]",
     (TLR + "test_the_filter_takes_the_treads_vibration_out_of_a_spin_but_not_the_turn",), True),
    ("IMU filter stepped once per tick", R,
     "        for _ in range(self.d[\"substeps\"]):\n            mujoco.mj_step(self.model, self.data)\n"
     "            self._imu_filter.update(",
     "        mujoco.mj_step(self.model, self.data, self.d[\"substeps\"])\n        if True:\n"
     "            self._imu_filter.update(",
     (TLR + "test_the_filter_steps_at_every_physics_step_and_starts_settled",), True),
    ("IMU filter not reset after settling", R, "        self.data.time = 0.0\n        self._reset_imu_filter()\n",
     "        self.data.time = 0.0\n", (TLR + "test_the_first_reading_is_the_settled_one_not_a_ramp_from_zero",), True),
    ("wheelstand reads a filter that never saw the held pose", WC,
     "    sim._reset_imu_filter()  # held still: the chip's filter has settled on this reading\n", "",
     (TWS + "Physics.test_the_model_balances_where_the_closed_form_says",), True),
    ("2.0 x scale typed by hand", "parameters-vector2.json", "        1.014,\n", "        1.02,\n",
     ("test_imu_fit.XScaleOnUnit3.test_the_2_0_table_holds_the_fitted_values_and_its_ranges_cover_the_bootstrap",), False),
    ("head sweeps keep the moving rows", IF, "if row[\"block\"] not in SWEEP_LEFT_OUT and not moving:",
     "if row[\"block\"] not in SWEEP_LEFT_OUT:",
     ("test_imu_fit.XScaleOnUnit3.test_rows_with_the_treads_or_the_gyro_moving_and_the_setup_sweep_are_left_out",), False),
    ("head sweeps keep the setup sweep", IF, 'SWEEP_LEFT_OUT = ("B0setup", "Transit0cw")', 'SWEEP_LEFT_OUT = ("Transit0cw",)',
     ("test_imu_fit.XScaleOnUnit3.test_rows_with_the_treads_or_the_gyro_moving_and_the_setup_sweep_are_left_out",), False),
    ("accelerometer offset dropped", R, ' + mg * v["accel_bias_mg"][i]', "",
     (TS + "RangeSensors.test_accelerometer_at_rest_reads_gravity_in_the_heads_pitched_frame_plus_the_offsets",), True),
    ("IMU fixed to the body, not the head", M,
     r"""        f'      <site name="imu" pos="{vec([c * MM for c in imu])}" quat="{vec(imu_quat)}" size="0.001"/>\n'
        f'    </body>\n'""",
     r"""        f'    </body>\n'
        f'    <site name="imu" pos="{vec([c * MM for c in imu])}" quat="{vec(imu_quat)}" size="0.001"/>\n'""",
     (TS + "RangeSensors.test_a_spin_shows_on_gyro_x_by_the_tangent_of_the_heads_pitch",), True),
    ("recording outside the skill read", MS, "if root.resolve() not in resolved.parents:", "if False:",
     (TMC + "Refusals.test_a_file_outside_the_skill_is_refused",), False),
    ("spin fraction read from gyro z alone", MS, '"whole": round(figures["whole"], 4)', '"whole": round(figures["gyro_z"], 4)',
     (TMC + "Recordings.test_spin_fractions_of_a_constructed_second_recording",), False),
    ("plan accepts any note", MS, "if len(note) > NOTE_CHARS or not NOTE_ALLOWED.match(note):", "if False:",
     (TMC + "Plan.test_a_note_that_would_break_the_shell_line_is_refused",), False),
    ("edge planned without a ruler distance", MS,
     'if "edge" in phases and (edge_start is None or not 5.0 <= edge_start <= 30.0 * 3.0):', "if False:",
     (TMC + "Plan.test_edge_needs_a_ruler_distance",), False),
    ("client protocol version not echoed", MS, '"protocolVersion": body.get("protocolVersion") or DEFAULT_PROTOCOL',
     '"protocolVersion": DEFAULT_PROTOCOL', (TMC + "Protocol.test_initialize_echoes_the_clients_protocol_version",), False),
    ("notifications answered", MS, "    if msg_id is None:\n        return None\n", "",
     (TMC + "Protocol.test_a_notification_gets_no_reply_and_an_unknown_method_gets_an_error",), False),
    ("phase space table loses a phase", MS, '    "arcs": ("a circle of about 150 mm radius", True),\n', "",
     (TMC + "Plan.test_every_recorder_phase_has_its_space_written_down",), False),
    ("remote command leaves the note unquoted", MS, 'f"NOTE=\'{note}\'"', 'f"NOTE={note}"',
     (TMC + "Plan.test_the_dark_spin_plan",), False),
    ("Windows command line keeps forward slashes", MS, "{PureWindowsPath(stdin)}", "{stdin}",
     (TMC + "Plan.test_the_dark_spin_plan",), False),
    ("line fit intercept with the wrong sign", RC, "return slope, my - slope * mx", "return slope, my + slope * mx",
     (TRC + "test_the_gyro_turns_with_the_head",), False),
    ("rear balance measured from the front axle", WS, 'math.atan2(com_x + g["wheel_base"], com_z - g["rear_radius"])',
     'math.atan2(com_x, com_z - g["rear_radius"])',
     (TWS + "Geometry.test_closed_form_matches_a_bisection_of_the_rotated_centre_of_mass",), False),
    ("IMU pitch offset ignored in the tilt", WS, 'angle = float(row["head"]) + math.radians(v["imu_pitch_offset_deg"])',
     'angle = float(row["head"])', (TWS + "Tilt.test_tilt_is_read_through_the_head_angle_offsets_and_scale",), False),
    ("accelerometer offsets not subtracted", WS,
     '((float(row["accel"][i]) - mg * v["accel_bias_mg"][i]) / v["accel_scale"][i] for i in range(3))',
     '(float(row["accel"][i]) / v["accel_scale"][i] for i in range(3))',
     (TWS + "Tilt.test_tilt_is_read_through_the_head_angle_offsets_and_scale",), False),
    ("roll read against uz alone", WS, "math.atan2(-uy, math.hypot(ux, uz))", "math.atan2(-uy, uz)",
     (TWS + "Tilt.test_tilt_is_read_through_the_head_angle_offsets_and_scale",), False),
    ("slow drift kept as one still stretch", WS,
     "max(abs(tilt[0] - first[0]), abs(tilt[1] - first[1])) > STRETCH_DRIFT_DEG",
     "max(abs(tilt[0] - first[0]), abs(tilt[1] - first[1])) > 1e9",
     (TWS + "Session_.test_a_slow_drift_under_the_gyro_threshold_is_cut_into_short_stretches",), False),
    ("trials off the reference pose used", WS, 'pose_ok = abs(head) <= v["head_angle_tol_deg"] and',
     'pose_ok = True or abs(head) <= v["head_angle_tol_deg"] and',
     (TWS + "Session_.test_a_constructed_session_brackets_both_angles_and_recovers_the_centre_of_mass",), False),
    ("bracket taken from the wrong ends", WS, "low, high = max(back), min(over)", "low, high = min(back), max(over)",
     (TWS + "Session_.test_a_constructed_session_brackets_both_angles_and_recovers_the_centre_of_mass",), False),
    ("a slow hand move counted as a release", WS, "    if peak < FALL_RATE_DPS:\n", "    if False:\n",
     (TWS + "Session_.test_a_constructed_session_brackets_both_angles_and_recovers_the_centre_of_mass",), False),
    ("wheelstand phase leaves the head where it is", HW,
     '    seconds = wheelstand_seconds()\n    b = robot.behavior\n    if not behaviour(robot, "head", lambda: b.set_head_angle(degrees(0.0)), deg=0.0):\n        return\n',
     '    seconds = wheelstand_seconds()\n    b = robot.behavior\n',
     (TRX + "Wheelstand.test_head_level_lift_down_then_nothing_commanded_while_listening",), False),
    ("wheelstand listen time unchecked", HW, 'assert 10.0 <= seconds <= MAX_LISTEN_S, "WHEELSTAND_S must be 10..300 s"',
     "assert True", (TRX + "Wheelstand.test_a_listen_time_out_of_range_is_refused_before_connecting",), False),
    ("wheelstand tool drops the ruler width", MS, "out = wheelstand.describe(path, width)",
     "out = wheelstand.describe(path)", (TMC + "Wheelstand.test_a_constructed_session_through_the_tool",), False),
    ("model accelerometer read mid-fall, not held still", WC, "    sim.data.qacc[:] = 0.0\n", "",
     (TWS + "Physics.test_the_model_balances_where_the_closed_form_says",), True),
    ("wheelie allowed before other phases", HW,
     'assert names[-1] == "wheelie" and "repeat" not in names, "wheelie must be the last phase, and not with repeat"',
     "assert True", (TRX + "Wheelie.test_wheelie_not_last_is_refused_before_connecting",), False),
    ("cube not waited for", HW, "        if cube is not None:\n            return cube\n",
     "        if True:\n            return cube\n",
     (TRX + "Wheelie.test_cube_then_the_robots_own_wheelie_then_only_recording",), False),
    ("the robot's answer to connect_cube not recorded", HW, '    emit("c", name="cube_connect", **_connect_fields(reply))\n',
     "", (TRX + "Wheelie.test_no_cube_means_nothing_is_commanded",), False),
    ("dock allowed before other phases", HW,
     'assert names[-1] == "dock" and "wheelie" not in names, "dock must be the last phase, and not with wheelie"',
     "assert True", (TRX + "Dock.test_dock_not_last_is_refused_before_connecting",), False),
    ("dock alone on the charger drives off first", HW, "    if st.is_on_charger and names not in ([\"dock\"], [\"nudge\"]):\n",
     "    if st.is_on_charger:\n", (TRX + "Dock.test_dock_alone_on_the_charger_commands_nothing",
                                   TRX + "Nudge.test_on_the_charger_it_commands_nothing"), False),
    ("endurance runs on past the low-battery voltage", HW,
     'if STATE["volts"] is not None and STATE["volts"] <= ENDURANCE_LOW_V:', "if False:",
     (TRX + "Endurance.test_it_stops_at_the_sdk_low_battery_voltage",), False),
    ("battery not read before each endurance cycle", HW, "            break\n        battery_read(robot)\n",
     "            break\n", (TRX + "Endurance.test_spins_above_the_top_speed_both_ways_with_the_battery_read_each_cycle",),
     False),
    ("battery read inside the hazard loop", HW, "    while time.monotonic() < end:\n        reason = hazard(robot)",
     "    while time.monotonic() < end:\n        battery_read(robot)\n        reason = hazard(robot)",
     (TRX + "Endurance.test_spins_above_the_top_speed_both_ways_with_the_battery_read_each_cycle",), False),
    ("endurance speed unchecked", HW, 'assert 100.0 <= speed <= MAX_ENDURANCE_MMPS, "ENDURANCE_MMPS must be 100..300 mm/s"',
     "assert True", (TRX + "Endurance.test_settings_out_of_range_are_refused_before_connecting",), False),
    ("VECTOR_IP not passed to the SDK", HW, '    extra = {"ip": IP} if IP else {}', "    extra = {}",
     (TRX + "Signals.test_the_address_comes_from_vector_ip_when_set_and_is_checked",), False),
    ("lift-spin phase skips the lift-up spins", HW, '(("down_a", 0.0), ("up", 1.0), ("down_b", 0.0))',
     '(("down_a", 0.0), ("down_b", 0.0))',
     (TRX + "LiftSpins.test_spins_with_the_lift_down_then_up_then_down_and_the_lift_left_down",), False),
    ("nudge keeps driving after the contact shows", HW,
     "            if robot.status.is_on_charger:\n                return\n    if wheels_for", "    if wheels_for",
     (TRX + "Nudge.test_it_stops_as_soon_as_the_charger_contact_shows",), False),
    ("a hang-up leaves the motors running", HW, "STOP_SIGNALS = (signal.SIGTERM, signal.SIGINT, signal.SIGHUP)",
     "STOP_SIGNALS = (signal.SIGTERM, signal.SIGINT)",
     (TRX + "Signals.test_a_hang_up_terminate_or_interrupt_stops_the_motors",), False),
    ("plan lets a wheelie run before other phases", MS,
     'if "wheelie" in phases and (phases[-1] != "wheelie" or "repeat" in phases):', "if False:",
     (TMC + "Plan.test_bad_phase_lists_and_output_names_are_refused",), False),
    ("peer capture asks for no control", CPS, "behavior_control_level=ControlPriorityLevel.DEFAULT_PRIORITY",
     "behavior_control_level=None", (TCPS + "test_it_holds_default_control_commands_nothing_and_streams_one_new_frame_per_tick",),
     False),
    ("peer capture runs without the control grant", CPS, "            if not wait_for_control(robot):", "            if False:",
     (TCPS + "test_control_never_granted_ends_without_lifting_quiet_or_starting_the_camera",), False),
    ("peer capture never puts quiet mode back", CPS, '        if restore:\n            quiet("POST", enable=True)\n',
     "        pass\n", (TCPS + "test_quiet_mode_on_is_lifted_only_while_control_is_held_and_put_back_before_release",
                       TCPS + "test_lost_control_stops_the_capture_and_quiet_mode_still_goes_back"), False),
    ("peer capture lifts quiet mode when told not to", CPS, '    if QUIET_LIFT == "1":', "    if True:",
     (TCPS + "test_quiet_lift_off_never_asks_about_quiet_mode",), False),
    ("peer capture ignores Cryosleep", CPS, "        if not cryosleep_clear():", "        if False:",
     (TCPS + "test_cryosleep_on_means_no_connection_and_no_quiet_change",), False),
    ("peer capture takes an unreachable pod as clear of Cryosleep", CPS,
     '        emit("c", name="cryo_check", error=("%s: %s" % (type(exc).__name__, exc))[:160])\n        return False',
     '        emit("c", name="cryo_check", error=("%s: %s" % (type(exc).__name__, exc))[:160])\n        return True',
     (TCPS + "test_cryosleep_unknown_means_no_connection_but_a_pod_without_the_endpoint_goes_ahead",), False),
    ("peer capture refuses a pod with no cryo endpoint", CPS, "        return exc.code == 404", "        return False",
     (TCPS + "test_cryosleep_unknown_means_no_connection_but_a_pod_without_the_endpoint_goes_ahead",), False),
    ("peer capture keeps going on a stalled camera", CPS, "        elif time.monotonic() - last_new >= STALL_S:",
     "        elif False:", (TCPS + "test_a_camera_that_stops_producing_new_frames_stops_the_capture",), False),
    ("peer capture keeps going after control is lost", CPS,
     '        if not has_control(robot):\n            return "control_lost", frames',
     '        if False:\n            return "control_lost", frames',
     (TCPS + "test_lost_control_stops_the_capture_and_quiet_mode_still_goes_back",), False),
    ("peer capture sends a frame it already sent", CPS, "image_id != last_id", "True",
     (TCPS + "test_a_camera_that_stops_producing_new_frames_stops_the_capture",), False),
    ("peer capture ignores a stop signal", CPS, '        if STATE["stop"]:\n            return "signal:" + STATE["stop"], frames',
     '        if False:\n            return "signal:" + STATE["stop"], frames', (TCPS + "test_a_stop_signal_stops_the_capture",),
     False),
    ("peer capture takes any IN_VIEW", CPS, 'assert IN_VIEW == ("none",) or (IN_VIEW and', "assert True or (IN_VIEW and",
     (TCPS + "test_settings_out_of_range_are_refused_before_anything_happens",), False),
    ("saver skips the hash check", SVC, 'if hashlib.sha256(data).hexdigest() != row.get("sha256") or data[:2]',
     "if data[:2]", (TSVC + "test_a_frame_whose_bytes_do_not_match_its_hash_stops_the_save",), False),
    ("saver writes into a folder that holds files", SVC,
     "    if os.path.exists(out) and (not os.path.isdir(out) or os.listdir(out)):", "    if False:",
     (TSVC + "test_it_refuses_an_output_folder_that_already_holds_anything",), False),
    ("saver accepts a repeated image id", SVC, ' or row.get("id") in ids:', ":",
     (TSVC + "test_frames_before_begin_or_a_repeated_image_id_stop_the_save",), False),
    ("dock pin contact ignores lateral alignment", "simulacrum/dock.py",
     "abs(float(delta_mm[1])) <= PIN_TOL_Y_MM", "True",
     ("test_dock.DockContactGeometry.test_nominal_offset_and_inverted_bounds",), False),
)
NEUTRAL = ("docstring reworded", R, '"""A simulated Vector on a table. One instance is one robot and one scene."""',
           '"""A simulated Vector on a table: one instance is one robot and one scene."""')


def _copy_tree(parent: Path, label: str) -> Path:
    dest = parent / label
    shutil.copytree(ROOT, dest, ignore=shutil.ignore_patterns("__pycache__", ".DS_Store", "node_modules", "dist"))
    assert (dest / "tests" / "run_all.py").is_file(), "copy is missing the test runner"
    assert (dest / "parameters.json").is_file(), "copy is missing the parameter table"
    return dest


def _swap(tree: Path, rel: str, old: str, new: str) -> Optional[str]:
    """Apply one text swap. Returns a set-up error, or None when the file really changed."""
    path = tree / rel
    before = path.read_text(encoding="utf-8")
    if old == new:
        return "old and new text are the same"
    if before.count(old) != 1:
        return f"needle occurs {before.count(old)} times in {rel}, expected exactly 1"
    after = before.replace(old, new)
    path.write_text(after, encoding="utf-8")
    return None if path.read_bytes() != before.encode("utf-8") else "file did not change"


def _run(tree: Path, only: Tuple[str, ...]) -> Dict[str, Any]:
    """Run the named tests (or all, when none are named) in a copy. Returns the runner's summary."""
    cmd = [sys.executable, str(tree / "tests" / "run_all.py")]
    for name in only:
        cmd += ["--only", name]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    try:
        done = subprocess.run(cmd, cwd=str(tree), env=env, capture_output=True, text=True, timeout=RUN_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        return {"setup_error": f"runner did not finish in {RUN_TIMEOUT_S} s"}
    lines = [line for line in done.stdout.splitlines() if line.startswith("SUMMARY ")]
    if len(lines) != 1:
        return {"setup_error": "no summary line; stderr: " + done.stderr.strip()[-300:]}
    summary = json.loads(lines[0][len("SUMMARY "):])
    summary["returncode"] = done.returncode
    return summary


def _verdict(summary: Dict[str, Any], expected: Tuple[str, ...]) -> str:
    """CAUGHT only for an assertion failure in an expected test that really ran."""
    if "setup_error" in summary:
        return "SETUP-ERROR: " + summary["setup_error"]
    if summary["ran"] != summary["declared"] or summary["ran"] != len(expected) or summary["skipped"]:
        return f"SETUP-ERROR: ran {summary['ran']} of {len(expected)} expected tests, skipped {summary['skipped']}"
    if summary["errored"]:
        return "SETUP-ERROR: test errored instead of failing: " + ", ".join(summary["errored"])
    return "CAUGHT" if summary["failed"] else "SURVIVED"


def _control(work: Path, arms: List[Tuple[Any, ...]]) -> Optional[str]:
    """The tests the arms rely on must pass on an untouched copy."""
    names = tuple(sorted({test for arm in arms for test in arm[4]}))
    assert len(names) > 0, "no tests to control"
    summary = _run(_copy_tree(work, "control"), names)
    if "setup_error" in summary:
        return summary["setup_error"]
    if summary["failed"] or summary["errored"] or summary["skipped"] or summary["ran"] != len(names):
        return f"control not green: {json.dumps(summary, sort_keys=True)}"
    print(f"control: {summary['ran']} tests named by the arms pass on an untouched copy")
    return None


def _neutral(work: Path) -> Optional[str]:
    """A change of wording only must leave the whole suite green."""
    name, rel, old, new = NEUTRAL
    tree = _copy_tree(work, "neutral")
    problem = _swap(tree, rel, old, new)
    if problem:
        return "neutral arm: " + problem
    summary = _run(tree, ())
    if "setup_error" in summary:
        return "neutral arm: " + summary["setup_error"]
    if summary["failed"] or summary["errored"] or summary["returncode"] != 0:
        return f"neutral arm ({name}) broke the suite: {json.dumps(summary, sort_keys=True)}"
    print(f"neutral ({name}): SURVIVED as required; {summary['ran']} ran, {summary['skipped']} skipped")
    return None


def _run_arms(work: Path, arms: List[Tuple[Any, ...]]) -> Dict[str, int]:
    counts = {"CAUGHT": 0, "SURVIVED": 0, "SETUP-ERROR": 0}
    assert len(arms) <= len(ARMS), "arm list grew"
    for index, (name, rel, old, new, tests, _) in enumerate(arms):
        tree = _copy_tree(work, f"arm{index:02d}")
        problem = _swap(tree, rel, old, new)
        verdict = "SETUP-ERROR: " + problem if problem else _verdict(_run(tree, tests), tests)
        counts[verdict.split(":")[0]] += 1
        print(f"arm {index + 1:2d} {name}: {verdict}")
    assert sum(counts.values()) == len(arms), "an arm was not counted"
    return counts


def main() -> int:
    arms = [arm for arm in ARMS if _common.HAVE_TRAINING or not arm[5]]
    not_run = len(ARMS) - len(arms)
    work = Path(tempfile.mkdtemp(prefix="vector-simulacrum-mutate-"))
    try:
        problem = _control(work, arms) or _neutral(work)
        if problem:
            print("SETUP-ERROR: " + problem)
            return 3
        counts = _run_arms(work, arms)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print(f"mutation result: {counts['CAUGHT']} of {len(arms)} arms caught, {counts['SURVIVED']} survived, "
          f"{counts['SETUP-ERROR']} set-up errors, {not_run} not run (need MuJoCo and the training packages); "
          f"control passed; "
          f"neutral change survived")
    if counts["SETUP-ERROR"]:
        return 3
    return 0 if counts["SURVIVED"] == 0 else 7


if __name__ == "__main__":
    sys.exit(main())
