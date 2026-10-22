#!/usr/bin/env python3
"""Offline visual prediction/actuator review. Never opens hardware or starts standard.

Future reference = accepted solved PnP of the same tracker epoch and face.
This measures visual consistency, not projectile accuracy or independent ground truth.
"""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import statistics


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def vector(value, size=3):
    return isinstance(value, list) and len(value) >= size and all(finite(x) for x in value[:size])


def stats(values):
    values = sorted(x for x in values if finite(x))
    if not values:
        return {"n": 0}
    absolute = sorted(abs(x) for x in values)
    return {"n": len(values), "mean": statistics.mean(values),
            "median": statistics.median(values), "min": values[0], "max": values[-1],
            "abs_p90": absolute[round((len(values) - 1) * .90)],
            "abs_p95": absolute[round((len(values) - 1) * .95)]}


def compare_yaw_zero_references(error_rad, correction_rad, tolerance_rad):
    """Same point/reference/trim, with the command-frame offset as a hypothesis."""
    if not all(finite(x) for x in (error_rad, correction_rad, tolerance_rad)) or tolerance_rad < 0:
        return None
    legacy = math.remainder(error_rad, math.tau)
    command = math.remainder(legacy + correction_rad, math.tau)
    return {"legacy_error_rad": legacy, "command_frame_error_rad": command,
            "legacy_pass": abs(legacy) <= tolerance_rad,
            "command_frame_pass": abs(command) <= tolerance_rad}


def analyze_yaw_zero_shadow(frames):
    """Read recorded continuous high-speed gates; no future labels or new requests."""
    rows, excluded = [], Counter()
    for f in frames:
        if f.get("rotation_mode") != "high_speed":
            continue
        if (f.get("gate_prediction_source") not in ("native_prediction", "fixed_center_prediction") or
                f.get("fire_branch") not in ("vehicle_prediction", "high_speed_prediction") or
                not f.get("native_prediction_valid")):
            excluded["not_native_predictive_gate"] += 1
            continue
        shadow = f.get("yaw_zero_shadow") or {}
        correction = shadow.get("command_correction_rad")
        if not finite(correction):
            boresight, impact = f.get("yaw_boresight_correction_rad"), f.get("yaw_impact_correction_rad")
            if finite(boresight) and finite(impact):
                correction = max(-math.radians(10), min(math.radians(10), boresight + impact))
        result = compare_yaw_zero_references(
            f.get("gate_error_rad"), correction, f.get("gate_tolerance_rad"))
        if result is None:
            excluded["missing_recorded_error_correction_or_tolerance"] += 1
            continue
        # A future schema or a stale shadow must not silently validate itself.
        if shadow and (not shadow.get("valid") or any(
                not finite(shadow.get(k)) or
                abs(math.remainder(shadow[k] - result[k], math.tau)) > 1e-9
                for k in ("legacy_error_rad", "command_frame_error_rad")) or any(
                    shadow.get(k) != result[k] for k in ("legacy_pass", "command_frame_pass"))):
            excluded["runtime_shadow_mismatch"] += 1
            continue
        rows.append({"run": f["run"], "frame_seq": f["frame_seq"],
                     "prediction_source": f.get("gate_prediction_source"),
                     "prediction_ns": f.get("gate_prediction_ns"), "face_id": f.get("gate_face_id"),
                     "log_time": f.get("log_time"), "request_shoot": bool(f.get("request_shoot")),
                     "omega_sign": "positive" if (f.get("omega_rad_s") or 0) > 0 else "negative",
                     "source": "runtime_shadow" if shadow else "recorded_gate_and_corrections",
                     "command_correction_deg": math.degrees(correction), **result})

    def summarize(selected):
        counts = Counter((r["legacy_pass"], r["command_frame_pass"]) for r in selected)
        return {"frames": len(selected), "both_pass": counts[True, True],
                "legacy_only_pass": counts[True, False],
                "command_frame_only_pass": counts[False, True], "neither_pass": counts[False, False],
                "legacy_error_deg": stats([math.degrees(r["legacy_error_rad"]) for r in selected]),
                "command_frame_error_deg": stats([math.degrees(r["command_frame_error_rad"]) for r in selected])}

    return {"basis": "same_recorded_native_point_and_reference_command_frame_hypothesis",
            "physical_bore_zero_verified": False, "new_requests_simulated": False,
            "all_compared": summarize(rows),
            "same_original_requests": summarize([r for r in rows if r["request_shoot"]]),
            "by_rotation_sign": {s: summarize([r for r in rows if r["omega_sign"] == s])
                                 for s in ("negative", "positive")},
            "exclusions": dict(excluded), "samples": rows,
            "limitation": "Yaw gate only; disagreement does not identify the physical zero or justify applying an offset. Other gates and timing are not replayed."}


def analyze_camera_policy(frames):
    """Application capture sequence is not an SDK loss counter or exposure clock."""
    rows = [f for f in frames if (f.get("camera_capture_sequence") or 0) > 0]
    groups = {}
    for policy in (False, True):
        selected = [f for f in rows if bool(f.get("camera_latest_selected")) == policy]
        groups["latest" if policy else "first_pending"] = {
            "frames": len(selected),
            "age_at_read_ms": stats([f.get("camera_age_at_read_ms") for f in selected]),
            "skipped_converted_frames": sum(f.get("camera_skipped_frames", 0) for f in selected),
            "imu_invalid": sum(not f.get("frame_imu_valid") for f in selected),
            "imu_extrapolated": sum(bool(f.get("frame_imu_extrapolated")) for f in selected),
            "imu_nearest_offset_ms": stats([f.get("frame_imu_nearest_offset_ms") for f in selected])}
    return {"status": "recorded" if rows else "no_capture_metadata", "groups": groups,
            "limitation": "Policy uses the preceding frame's motion classification. Host receipt age includes the configured offset; physical exposure time and SDK buffering remain unverified."}


def read_frames(paths):
    frames, seen, parse_errors = [], set(), 0
    legacy_priors, legacy_conflicts = {}, set()
    for path in paths:
        with path.open(encoding="utf-8", errors="replace") as stream:
            for line in stream:
                if "[INFANTRY_DIAG] {" in line:
                    try:
                        diagnostic = json.loads(line.split("[INFANTRY_DIAG] ", 1)[1])
                        key = (diagnostic.get("run"), diagnostic.get("frame_seq"))
                        if key in legacy_priors and legacy_priors[key] != diagnostic:
                            legacy_conflicts.add(key)
                        legacy_priors[key] = diagnostic
                    except ValueError:
                        parse_errors += 1
                    continue
                if "[SHOT_DEBUG] " not in line:
                    continue
                try:
                    item = json.loads(line.split("[SHOT_DEBUG] ", 1)[1])
                except ValueError:
                    parse_errors += 1
                    continue
                if item.get("kind") != "prediction_frame":
                    continue
                key = (item.get("run"), item.get("frame_seq"))
                if key in seen:
                    continue  # App log and launcher console may contain identical records.
                if not finite(item.get("frame_ns")):
                    parse_errors += 1
                    continue
                seen.add(key)
                item["log_time"] = line[1:24] if line.startswith("[20") else None
                frames.append(item)
    for f in frames:
        key = (f["run"], f["frame_seq"])
        if "state_before_update" in f or key in legacy_conflicts:
            continue
        diagnostic = legacy_priors.get(key, {})
        if not diagnostic.get("tracker_update_attempted") or any(diagnostic.get(k) for k in [
                "tracker_initialized_this_frame", "tracker_nis_reset", "tracker_update_diverged"]):
            continue
        state = diagnostic.get("tracker_state_prior") or {}
        keys = ["center_x_m", "center_vx_mps", "center_y_m", "center_vy_mps", "center_z_m", "center_vz_mps",
                "body_yaw_deg", "raw_omega_rad_s", "r1_m", "r2_delta_m", "height_delta_m"]
        if all(finite(state.get(k)) for k in keys):
            f["state_before_update"] = [state[k] for k in keys]
            f["state_before_update"][6] = math.radians(state["body_yaw_deg"])
            f["state_before_update_source"] = "same_frame_sampled_legacy_diagnostic"
    return sorted(frames, key=lambda f: (f["run"], f["frame_ns"])), parse_errors


def series(points):
    """Ambiguous same-time entries are excluded, never arbitrarily selected."""
    grouped = defaultdict(list)
    for timestamp, value in points:
        grouped[timestamp].append(value)
    points = sorted((t, v[0]) for t, v in grouped.items() if len(v) == 1)
    return [p[0] for p in points], [p[1] for p in points]


def interpolate(data, timestamp, max_offset_ns, angular=False):
    times, values = data
    i = bisect_left(times, timestamp)
    if i < len(times) and times[i] == timestamp:
        return values[i], 0, None
    if i == 0 or i == len(times):
        return None, None, "no_bracket"
    t0, t1 = times[i - 1], times[i]
    if timestamp - t0 > max_offset_ns or t1 - timestamp > max_offset_ns or t1 - t0 > max_offset_ns * 2:
        return None, None, "observation_gap"
    fraction = (timestamp - t0) / (t1 - t0)
    v0, v1 = values[i - 1], values[i]
    result = [a + fraction * (math.remainder(b - a, math.tau) if angular else b - a)
              for a, b in zip(v0, v1)]
    return result, (t1 - t0) / 1e6, None


def summarize_residuals(rows):
    fields = ["yaw_error_deg", "pitch_error_deg", "transverse_error_cm", "xyz_error_cm",
              "phase_error_deg", "horizon_ms", "reference_bracket_ms"]
    return {field: stats([r[field] for r in rows]) for field in fields}


def rigid_face(state, face, count):
    """Exact Target::h_armor_xyz geometry; no independent truth or motion fitting."""
    if not vector(state, 11) or not isinstance(count, int) or count < 1 or not isinstance(face, int) or not 0 <= face < count:
        return None
    angle = state[6] + face * math.tau / count
    alternate = count == 4 and face % 2 == 1
    radius = state[8] + (state[9] if alternate else 0)
    return [state[0] - radius * math.cos(angle), state[2] - radius * math.sin(angle),
            state[4] + (state[10] if alternate else 0), math.remainder(angle, math.tau)]


def point_error(predicted, reference):
    py, ry = math.atan2(predicted[1], predicted[0]), math.atan2(reference[1], reference[0])
    pp = math.atan2(predicted[2], math.hypot(*predicted[:2]))
    rp = math.atan2(reference[2], math.hypot(*reference[:2]))
    delta = [predicted[i] - reference[i] for i in range(3)]
    return {"yaw_error_deg": math.degrees(math.remainder(py - ry, math.tau)),
            "pitch_error_deg": math.degrees(pp - rp),
            "phase_error_deg": math.degrees(math.remainder(predicted[3] - reference[3], math.tau)),
            "xyz_error_cm": 100 * math.sqrt(sum(d * d for d in delta)),
            "transverse_error_cm": 100 * (-math.sin(ry) * delta[0] + math.cos(ry) * delta[1])}


def summarize_rows(rows):
    keys = sorted({k for row in rows for k in row if finite(row[k])})
    return {k: stats([row.get(k) for row in rows]) for k in keys}


def valid_observations(frame):
    if frame.get("observations_truncated"):
        return []
    counts = Counter(o.get("face_id") for o in frame.get("observations", []))
    return sorted([o for o in frame.get("observations", [])
            if isinstance(o.get("face_id"), int) and o["face_id"] >= 0 and counts[o["face_id"]] == 1
            and vector(o.get("xyz_world_m")) and finite(o.get("body_yaw_rad"))], key=lambda o: o["face_id"])


def observation_group(frame, observation):
    incidence = math.degrees(math.remainder(observation["body_yaw_rad"] -
                            math.atan2(observation["xyz_world_m"][1], observation["xyz_world_m"][0]), math.tau))
    return (f"run{frame['run']}/epoch{frame.get('track_epoch')}/face{observation['face_id']}/"
            f"incidence{math.floor(incidence / 20) * 20}/accepted{len(valid_observations(frame))}/"
            f"fallback{int(bool(observation.get('identity_fallback')))}")


def dual_center_geometry(left, right, state, count=4, yaw_key="body_yaw_rad"):
    """Horizontal two-plate checks, with recorded radii and no online state fitting.

    Positions determine a common phase from their chord, independently of PnP yaw.
    Individual PnP yaws instead determine two implied centers. Neither is truth.
    """
    if count != 4 or not vector(state, 11):
        raise ValueError("missing_four_plate_state")
    ids = [left.get("face_id"), right.get("face_id")]
    if any(type(i) is not int or not 0 <= i < count for i in ids) or ids[0] == ids[1]:
        raise ValueError("invalid_or_duplicate_face")
    if any(not vector(o.get("xyz_world_m")) or not finite(o.get(yaw_key)) for o in [left, right]):
        raise ValueError("missing_position_or_yaw")
    radii = [state[8] + (state[9] if i % 2 else 0) for i in ids]
    if any(r <= 0 for r in radii):
        raise ValueError("nonpositive_recorded_radius")
    offsets = [i * math.tau / count for i in ids]
    points = [o["xyz_world_m"][:2] for o in [left, right]]
    yaws = [o[yaw_key] for o in [left, right]]
    chord = [points[1][j] - points[0][j] for j in range(2)]
    template = [radii[0] * fn(offsets[0]) - radii[1] * fn(offsets[1])
                for fn in [math.cos, math.sin]]
    length, expected = math.hypot(*chord), math.hypot(*template)
    if min(length, expected) < 1e-8:
        raise ValueError("degenerate_position_chord")
    phase = math.remainder(math.atan2(chord[1], chord[0]) - math.atan2(template[1], template[0]), math.tau)
    centers = [[p[j] + r * fn(yaw) for j, fn in enumerate([math.cos, math.sin])]
               for p, r, yaw in zip(points, radii, yaws)]
    position_centers = [[p[j] + r * fn(phase + offset) for j, fn in enumerate([math.cos, math.sin])]
                        for p, r, offset in zip(points, radii, offsets)]
    center = [sum(c[j] for c in position_centers) / 2 for j in range(2)]
    yaw_center = [sum(c[j] for c in centers) / 2 for j in range(2)]
    bearing = math.atan2(sum(p[1] for p in points), sum(p[0] for p in points))
    gap = [centers[1][j] - centers[0][j] for j in range(2)]
    row = {"orientation_center_gap_cm": 100 * math.dist(*centers),
           "orientation_center_gap_radial_cm": 100 * (math.cos(bearing) * gap[0] + math.sin(bearing) * gap[1]),
           "orientation_center_gap_transverse_cm": 100 * (-math.sin(bearing) * gap[0] + math.cos(bearing) * gap[1]),
           "recorded_left_radius_m": radii[0], "recorded_right_radius_m": radii[1],
           "position_chord_length_error_cm": 100 * (length - expected),
           "position_phase_minus_left_yaw_deg": math.degrees(math.remainder(phase + offsets[0] - yaws[0], math.tau)),
           "position_phase_minus_right_yaw_deg": math.degrees(math.remainder(phase + offsets[1] - yaws[1], math.tau)),
           "position_center_minus_prior_cm": 100 * math.dist(center, [state[0], state[2]]),
           "yaw_center_minus_position_center_cm": 100 * math.dist(center, yaw_center)}
    # If both observed orientations were exact, solve their two center equations
    # for separate radii. This interpolates two observations, not a validation fit.
    a, b = math.cos(yaws[0]), -math.cos(yaws[1])
    c, d = math.sin(yaws[0]), -math.sin(yaws[1])
    determinant = a * d - b * c
    row["radius_solve_abs_sine"] = abs(determinant)
    if abs(determinant) >= .1:
        fitted = [(chord[0] * d - b * chord[1]) / determinant,
                  (a * chord[1] - chord[0] * c) / determinant]
        row.update(left_implied_radius_m=fitted[0], right_implied_radius_m=fitted[1],
                   left_implied_minus_recorded_radius_cm=100 * (fitted[0] - radii[0]),
                   right_implied_minus_recorded_radius_cm=100 * (fitted[1] - radii[1]),
                   implied_radius_nonpositive=int(min(fitted) <= 0))
    return row, center


def solve_small_system(matrix, values):
    """Pivoted elimination for the offline four-parameter center hypothesis."""
    n = len(values)
    rows = [list(a) + [b] for a, b in zip(matrix, values)]
    scale = max((abs(x) for a in matrix for x in a), default=0.)
    for col in range(n):
        pivot = max(range(col, n), key=lambda i: abs(rows[i][col]))
        if abs(rows[pivot][col]) <= max(1., scale) * 1e-10:
            raise ValueError("rank_deficient_constant_center_fit")
        rows[col], rows[pivot] = rows[pivot], rows[col]
        factor = rows[col][col]
        rows[col] = [x / factor for x in rows[col]]
        for i in range(n):
            if i != col:
                factor = rows[i][col]
                rows[i] = [x - factor * y for x, y in zip(rows[i], rows[col])]
    return [row[-1] for row in rows]


def constant_center_holdout(frames, yaw_key="body_yaw_rad"):
    """Fit Cx,Cy,r_even,r_odd under a constant-center hypothesis, not a controller.

    Alternate one-second blocks train/test. Runs/epochs must be split by caller.
    Only dual-observation frames are used, each plate once, without PnP re-solving.
    """
    if not frames:
        return {"status": "excluded", "reason": "no_dual_frames"}
    origin = min(f["frame_ns"] for f in frames)
    train, test = [], []
    for f in frames:
        obs = valid_observations(f)
        if any(not finite(o.get(yaw_key)) for o in obs):
            continue  # Raw-yaw mode never substitutes optimized yaw.
        destination = train if int((f["frame_ns"] - origin) // 1_000_000_000) % 2 == 0 else test
        destination.extend((f, o) for o in obs)
    metadata = {"train_observations": len(train), "test_observations": len(test),
                "train_frames": len({f["frame_seq"] for f, _ in train}),
                "test_frames": len({f["frame_seq"] for f, _ in test}),
                "split": "alternating_one_second_blocks"}
    if min(metadata["train_frames"], metadata["test_frames"]) < 6:
        return dict(metadata, status="excluded", reason="insufficient_train_or_test_frames")
    normal, rhs = [[0.] * 4 for _ in range(4)], [0.] * 4
    for _, o in train:
        yaw, parity = o[yaw_key], o["face_id"] % 2
        for axis, fn in enumerate([math.cos, math.sin]):
            design = [float(axis == 0), float(axis == 1), 0., 0.]
            design[2 + parity] = -fn(yaw)
            for i in range(4):
                rhs[i] += design[i] * o["xyz_world_m"][axis]
                for j in range(4):
                    normal[i][j] += design[i] * design[j]
    try:
        fit = solve_small_system(normal, rhs)
    except ValueError as error:
        return dict(metadata, status="excluded", reason=str(error))
    if not all(finite(x) for x in fit) or min(fit[2:]) <= 0:
        return dict(metadata, status="excluded", reason="nonpositive_or_invalid_fitted_radius", fitted_parameters=fit)
    metrics, paired_fit, paired_prior = [], [], []
    for f, o in test:
        radius, yaw = fit[2 + o["face_id"] % 2], o[yaw_key]
        p = [fit[0] - radius * math.cos(yaw), fit[1] - radius * math.sin(yaw), o["xyz_world_m"][2], yaw]
        error = point_error(p, o["xyz_world_m"][:3] + [o["body_yaw_rad"]])
        row = {"xy_error_cm": error["xyz_error_cm"], "yaw_error_deg": error["yaw_error_deg"]}
        metrics.append(row)
        prior = rigid_face(f.get("state_before_update"), o["face_id"], 4)
        if prior is not None:
            e = point_error(prior, o["xyz_world_m"][:3] + [o["body_yaw_rad"]])
            paired_fit.append(row)
            paired_prior.append({"xy_error_cm": 100 * math.dist(prior[:2], o["xyz_world_m"][:2]),
                                 "yaw_error_deg": e["yaw_error_deg"]})
    return dict(metadata, status="ok", center_world_xy_m=fit[:2], radii_even_odd_m=fit[2:],
                heldout_residuals=summarize_rows(metrics),
                paired_heldout_fit=summarize_rows(paired_fit), paired_heldout_prior=summarize_rows(paired_prior))


def correlation(rows, x, y):
    pairs = [(r[x], r[y]) for r in rows if finite(r.get(x)) and finite(r.get(y))]
    if len(pairs) < 3:
        return {"n": len(pairs), "pearson": None}
    mx, my = (statistics.mean(p[i] for p in pairs) for i in range(2))
    xx = sum((p[0] - mx) ** 2 for p in pairs)
    yy = sum((p[1] - my) ** 2 for p in pairs)
    return {"n": len(pairs), "pearson": sum((a - mx) * (b - my) for a, b in pairs) / math.sqrt(xx * yy)
            if min(xx, yy) > 1e-20 else None}


def analyze_dual_geometry(frames):
    pairs, raw_pairs, samples = [], [], []
    groups, temporal, fit_frames = defaultdict(list), defaultdict(list), defaultdict(list)
    view_updates, exclusions = defaultdict(list), Counter()
    for f in frames:
        if f.get("observations_truncated"):
            exclusions["truncated_observations"] += 1
            continue
        if not f.get("target_present") or not f.get("frame_imu_valid") or not f.get("tracker_converged"):
            exclusions["untrusted_or_unsynced"] += 1
            continue
        if f.get("armor_count") != 4:
            exclusions["not_four_plate_target"] += 1
            continue
        obs = valid_observations(f)
        if len(obs) < 2:
            exclusions["fewer_than_two_valid_faces"] += 1
            continue
        if any(o["face_id"] >= 4 for o in obs):
            exclusions["face_out_of_range"] += 1
            continue
        key = (f["run"], f.get("track_epoch"))
        fit_frames[key].append(f)
        prior, post = f.get("state_before_update"), f.get("state_at_frame")
        if not vector(prior, 11):
            exclusions["missing_prior_radii"] += 1
            continue
        frame_centers = []
        for i, left in enumerate(obs):
            for right in obs[i + 1:]:
                try:
                    row, center = dual_center_geometry(left, right, prior)
                except ValueError as error:
                    exclusions[str(error)] += 1
                    continue
                if row["radius_solve_abs_sine"] < .1:
                    exclusions["ill_conditioned_implied_radius_pair"] += 1
                pairs.append(row)
                frame_centers.append(center)
                sign = "positive" if (f.get("omega_rad_s") or 0) > 0 else "negative"
                group = f"run{key[0]}/epoch{key[1]}/rotation_{sign}/pair{left['face_id']}-{right['face_id']}"
                groups[group].append(row)
                for side, o in [("left", left), ("right", right)]:
                    groups[side + "/" + observation_group(f, o)].append(row)
                sample = dict(row, run=key[0], track_epoch=key[1], frame_seq=f["frame_seq"],
                              frame_ns=f["frame_ns"], log_time=f.get("log_time"), face_ids=[left["face_id"], right["face_id"]],
                              position_center_world_xy_m=center)
                try:
                    raw, _ = dual_center_geometry(left, right, prior, yaw_key="pnp_yaw_before_optimization_rad")
                    raw_pairs.append(raw)
                    sample["raw_yaw_geometry"] = raw
                except ValueError as error:
                    exclusions["raw_" + str(error)] += 1
                samples.append(sample)
        if frame_centers:
            center = [statistics.mean(c[j] for c in frame_centers) for j in range(2)]
            temporal[key].append((center, [prior[0], prior[2]]))
        if vector(post, 11):
            for o in obs:
                model = rigid_face(prior, o["face_id"], 4)
                reference = o["xyz_world_m"][:3] + [o["body_yaw_rad"]]
                error = point_error(model, reference)
                bearing = math.atan2(reference[1], reference[0])
                dx, dy = post[0] - prior[0], post[2] - prior[2]
                update = {"prior_phase_residual_deg": error["phase_error_deg"],
                          "prior_yaw_residual_deg": error["yaw_error_deg"],
                          "center_update_transverse_cm": 100 * (-math.sin(bearing) * dx + math.cos(bearing) * dy),
                          "center_update_radial_cm": 100 * (math.cos(bearing) * dx + math.sin(bearing) * dy),
                          "phase_update_deg": math.degrees(math.remainder(post[6] - prior[6], math.tau)),
                          "radius_update_cm": 100 * (post[8] - prior[8] + ((post[9] - prior[9]) if o["face_id"] % 2 else 0))}
                view_updates[observation_group(f, o)].append(update)
    spread = {}
    for key, rows in temporal.items():
        spread[f"run{key[0]}/epoch{key[1]}"] = {"frames": len(rows)}
        for i, label in enumerate(["position_pair_center", "recorded_prior_center"]):
            spread[f"run{key[0]}/epoch{key[1]}"][label + "_xy_sd_cm"] = [
                100 * statistics.pstdev(row[i][j] for row in rows) for j in range(2)]
    return {"pair_count": len(pairs), "optimized_yaw_geometry": summarize_rows(pairs),
            "raw_yaw_geometry": summarize_rows(raw_pairs), "groups": {k: summarize_rows(v) for k, v in groups.items()},
            "within_epoch_center_spread": spread,
            "constant_center_holdout": {f"run{k[0]}/epoch{k[1]}": {
                "optimized_yaw": constant_center_holdout(v),
                "raw_yaw": constant_center_holdout(v, "pnp_yaw_before_optimization_rad")} for k, v in fit_frames.items()},
            "view_angle_updates": {k: {"metrics": summarize_rows(v),
                "phase_residual_vs_center_update": correlation(v, "prior_phase_residual_deg", "center_update_transverse_cm"),
                "phase_update_vs_center_update": correlation(v, "phase_update_deg", "center_update_transverse_cm")}
                for k, v in view_updates.items()},
            "exclusions": dict(exclusions), "pair_samples": samples,
            "interpretation": ["Four-plate XY geometry only; prior radii are recorded before this observation update.",
                "Chord phase uses positions plus recorded radii; center closure uses individual PnP yaws. Both share calibration and geometry errors.",
                "Implied radii interpolate two orientations exactly; negative values are evidence, not clamped. abs(sin yaw separation)<0.1 is excluded from this solve only.",
                "Constant-center fitting is an offline no-translation hypothesis with one radius per parity, not an online recommendation. Heldout blocks share calibration and temporal correlations.",
                "Center spread is descriptive, not measured translation. Face ids and parity remain tracker-relative.",
                "View groups use observed angles and accepted plates; correlations do not establish causation."]}


def analyze_attribution(frames, residuals):
    """Separate current geometry consistency, future error and actual command chain."""
    prior, posterior, dual, commands = [], [], [], defaultdict(list)
    grouped, forecast_groups = defaultdict(list), defaultdict(list)
    paired_prior, paired_posterior = [], []
    incremental, exclusions, prior_sources = [], Counter(), Counter()
    index = {(f["run"], f["frame_seq"]): f for f in frames}
    for f in frames:
        if f.get("observations_truncated"):
            exclusions["truncated_observations"] += 1
            continue
        if not f.get("target_present") or not f.get("frame_imu_valid") or not f.get("tracker_converged"):
            exclusions["current_model_untrusted_or_unsynced"] += 1
            continue
        observations = valid_observations(f)
        for o in observations:
            reference = o["xyz_world_m"][:3] + [o["body_yaw_rad"]]
            pair = {}
            for state_key, destination in [("state_before_update", prior), ("state_at_frame", posterior)]:
                model = rigid_face(f.get(state_key), o["face_id"], f.get("armor_count"))
                if model is None:
                    exclusions[state_key + "_missing_or_invalid"] += 1
                    continue
                error = point_error(model, reference)
                pair[state_key] = error
                destination.append(error)
                if state_key == "state_before_update":
                    prior_sources[f.get("state_before_update_source", "continuous_schema2")] += 1
                grouped[state_key + "/" + observation_group(f, o)].append(error)
            if len(pair) == 2:
                paired_prior.append(pair["state_before_update"])
                paired_posterior.append(pair["state_at_frame"])
            if finite(o.get("pnp_yaw_before_optimization_rad")):
                grouped["yaw_optimization/" + observation_group(f, o)].append({
                    "change_deg": math.degrees(math.remainder(o["body_yaw_rad"] -
                                                  o["pnp_yaw_before_optimization_rad"], math.tau))})
        for i, left in enumerate(observations):
            for right in observations[i + 1:]:
                count = f.get("armor_count")
                if not isinstance(count, int) or count < 1 or max(left["face_id"], right["face_id"]) >= count:
                    continue
                # Remove only known face offsets: no filter state needed for this check.
                phase = math.remainder(left["body_yaw_rad"] - right["body_yaw_rad"] -
                                       (left["face_id"] - right["face_id"]) * math.tau / count, math.tau)
                dual.append({"equivalent_body_phase_difference_deg": math.degrees(phase)})
        if f.get("tx_ok") and f.get("tx_control"):
            row = {}
            for key, first, second in [
                    ("final_rate_change_deg", f.get("yaw_after_final_rate_rad"), f.get("yaw_before_final_rate_rad")),
                    ("output_minus_native_yaw_deg", f.get("output_yaw_rad"), f.get("native_yaw_rad") if f.get("native_prediction_valid") else None)]:
                if finite(first) and finite(second):
                    row[key] = math.degrees(math.remainder(first - second, math.tau))
            for axis in ["yaw", "pitch"]:
                scale, output, packed = f.get("tx_" + axis + "_scale"), f.get("output_" + axis + "_rad"), f.get("tx_raw_" + axis + "_rad")
                if all(finite(x) for x in [scale, output, packed]) and scale != 0:
                    row["packed_minus_output_" + axis + "_deg"] = math.degrees(math.remainder(packed / scale - output, math.tau))
            commands["shoot_high" if f.get("tx_shoot") else "shoot_low"].append(row)
    for row in residuals:
        f = index[(row["run"], row["frame_seq"])]
        matching = [o for o in valid_observations(f) if o["face_id"] == row["face_id"]]
        forecast_groups[f"run{row['run']}/epoch{row['track_epoch']}/face{row['face_id']}/accepted{len(valid_observations(f))}"].append(row)
        if not matching:
            exclusions["future_face_not_observed_at_query"] += 1
            continue
        o = matching[0]
        forecast_groups[observation_group(f, o)].append(row)
        current = rigid_face(f.get("state_at_frame"), row["face_id"], f.get("armor_count"))
        if current is None:
            continue
        present = point_error(current, o["xyz_world_m"][:3] + [o["body_yaw_rad"]])
        incremental.append({axis + "_future_minus_current_deg": math.degrees(math.remainder(
            math.radians(row[axis + "_error_deg"] - present[axis + "_error_deg"]), math.tau))
                            for axis in ["yaw", "phase"]})
    return {"current_prior_vs_pnp": summarize_rows(prior),
            "current_posterior_vs_pnp": summarize_rows(posterior),
            "paired_current_prior_vs_pnp": summarize_rows(paired_prior),
            "paired_current_posterior_vs_pnp": summarize_rows(paired_posterior),
            "current_groups": {k: summarize_rows(v) for k, v in grouped.items()},
            "future_groups": {k: summarize_residuals(v) for k, v in forecast_groups.items()},
            "dual_face_same_frame": summarize_rows(dual),
            "future_minus_current_error": summarize_rows(incremental),
            "command_chain": {k: summarize_rows(v) for k, v in commands.items()},
            "prior_observation_sources": dict(prior_sources),
            "exclusions": dict(exclusions),
            "interpretation": ["Prior is after temporal prediction, before this frame's observation update; posterior already uses PnP.",
                               "Future minus current error includes observation noise; it is not isolated physical prediction error.",
                               "Incidence uses observed PnP yaw; accepted counts are not independent visible-plate counts.",
                               "Output minus native yaw includes deliberate centre following and boresight correction; it is not an accuracy metric.",
                               "Equivalent dual-face phase differences can reflect PnP geometry OR association; no automatic root-cause decision."]}


def calibration_context(frames):
    contexts, conflicts = {}, set()
    for f in frames:
        value = f.get("calibration")
        if value:
            if f["run"] in contexts and contexts[f["run"]] != value:
                conflicts.add(f["run"])
            contexts[f["run"]] = value
    return contexts, conflicts


def replay_pnp_observation(frame, observation, calibration, cv, np):
    """Recompute only offline; inspect both IPPE solutions and current fixed-pitch model."""
    if calibration.get("solver_model") != "ippe_fixed_pitch_yaw_grid_v1":
        raise ValueError("unsupported_solver_model")
    for key, length in [("camera_matrix", 9), ("distort_coeffs", 5), ("R_camera2gimbal", 9), ("t_camera2gimbal", 3)]:
        if not vector(calibration.get(key), length) or len(calibration[key]) != length:
            raise ValueError("invalid_calibration")
    corners = observation.get("corners_px")
    if not isinstance(corners, list) or len(corners) != 4 or not all(vector(p, 2) and len(p) == 2 for p in corners):
        raise ValueError("missing_or_invalid_corners")
    rotation = frame.get("solver_R_gimbal2world_rowmajor")
    if not vector(rotation, 9) or len(rotation) != 9:
        raise ValueError("missing_or_invalid_frame_rotation")
    k = np.asarray(calibration["camera_matrix"], dtype=float).reshape(3, 3)
    dist = np.asarray(calibration["distort_coeffs"], dtype=float)
    rcg = np.asarray(calibration["R_camera2gimbal"], dtype=float).reshape(3, 3)
    rgw = np.asarray(rotation, dtype=float).reshape(3, 3)
    if any(not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or abs(np.linalg.det(r) - 1) > 1e-5 for r in [rcg, rgw]):
        raise ValueError("non_rotation_matrix")
    tcg = np.asarray(calibration["t_camera2gimbal"], dtype=float)
    typ, name = observation.get("type"), observation.get("name")
    if typ not in [0, 1] or not isinstance(name, int) or not 0 <= name <= 7:
        raise ValueError("invalid_armor_identity")
    # Solver's compiled geometry and point order, versioned by solver_model above.
    width, height = (.230 if typ == 0 else .135), .056
    obj = np.array([[0, width/2, height/2], [0, -width/2, height/2],
                    [0, -width/2, -height/2], [0, width/2, -height/2]], dtype=np.float32)
    image = np.asarray(corners, dtype=np.float32)
    ok, rvec, tvec = cv.solvePnP(obj, image, k, dist, flags=cv.SOLVEPNP_IPPE)
    if not ok:
        raise ValueError("solve_pnp_failed")
    world = rgw @ (rcg @ tvec.reshape(3) + tcg)
    raw_rotation = rgw @ rcg @ cv.Rodrigues(rvec)[0]
    raw_yaw = math.atan2(raw_rotation[1, 0], raw_rotation[0, 0])
    logged = observation["xyz_world_m"]
    row = {"replay_xyz_difference_cm": float(np.linalg.norm(world - logged) * 100)}
    projected = cv.projectPoints(obj, rvec, tvec, k, dist)[0].reshape(4, 2)
    row["ippe_reprojection_rms_px"] = float(np.sqrt(np.mean(np.sum((projected - image)**2, axis=1))))
    logged_raw = observation.get("pnp_yaw_before_optimization_rad")
    if finite(logged_raw):
        row["replay_raw_yaw_difference_deg"] = math.degrees(math.remainder(raw_yaw - logged_raw, math.tau))
    balance = typ == 0 and name in [2, 3, 4]
    def fixed_pitch_points(yaw):
        pitch = math.radians(-15 if name == 6 else 15)
        cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
        rot = np.array([[cy*cp, -sy, cy*sp], [sy*cp, cy, sy*sp], [-sp, 0, cp]])
        cr = rcg.T @ rgw.T @ rot
        ct = rcg.T @ (rgw.T @ np.asarray(logged) - tcg)
        return cv.projectPoints(obj, cv.Rodrigues(cr)[0], ct, k, dist)[0].reshape(4, 2)
    if not balance:
        logged_points = fixed_pitch_points(observation["body_yaw_rad"])
        row["optimized_fixed_pitch_reprojection_rms_px"] = float(np.sqrt(np.mean(np.sum((logged_points-image)**2, axis=1))))
        gimbal_yaw = math.atan2(rgw[1, 0], rgw[0, 0])
        candidates = [math.remainder(gimbal_yaw + math.radians(i-70), math.tau) for i in range(140)]
        best = min(candidates, key=lambda yaw: float(np.linalg.norm(fixed_pitch_points(yaw)-image, axis=1).sum()))
        row["replay_optimized_yaw_difference_deg"] = math.degrees(math.remainder(best-observation["body_yaw_rad"], math.tau))
    solutions = cv.solvePnPGeneric(obj, image, k, dist, flags=cv.SOLVEPNP_IPPE)
    details = []
    for rv, tv in zip(solutions[1], solutions[2]):
        rot = cv.Rodrigues(rv)[0]
        proj = cv.projectPoints(obj, rv, tv, k, dist)[0].reshape(4, 2)
        wy = rgw @ rcg @ rot
        details.append({"all_corners_in_front": bool(np.all((rot @ obj.T + tv.reshape(3, 1))[2] > 0)),
                        "rms_px": float(np.sqrt(np.mean(np.sum((proj-image)**2, axis=1)))),
                        "yaw_rad": math.atan2(wy[1, 0], wy[0, 0])})
    if len(details) >= 2 and all(d["all_corners_in_front"] for d in details[:2]):
        row["ippe_second_minus_first_rms_px"] = details[1]["rms_px"] - details[0]["rms_px"]
        row["ippe_solution_yaw_separation_deg"] = abs(math.degrees(math.remainder(details[1]["yaw_rad"]-details[0]["yaw_rad"], math.tau)))
    return row


def analyze_pnp_replay(frames, contexts, conflicts, stride=10):
    try:
        import cv2 as cv
        import numpy as np
    except ImportError as error:
        return {"status": "unavailable", "reason": str(error), "required": "OpenCV Python and NumPy; offline only"}
    rows, groups, excluded = [], defaultdict(list), Counter()
    sampled = 0
    for i, f in enumerate(frames):
        if i % stride:
            continue
        sampled += 1
        if f["run"] in conflicts or f["run"] not in contexts:
            excluded["conflicting_calibration" if f["run"] in conflicts else "missing_calibration"] += 1
            continue
        if not f.get("frame_imu_valid") or not f.get("tracker_converged"):
            excluded["untrusted_or_unsynced"] += 1
            continue
        for observation in valid_observations(f):
            try:
                row = replay_pnp_observation(f, observation, contexts[f["run"]], cv, np)
                if not all(finite(x) for x in row.values()):
                    raise ValueError("nonfinite_replay")
            except (ValueError, cv.error, KeyError, TypeError) as error:
                excluded[str(error) if isinstance(error, ValueError) else type(error).__name__] += 1
                continue
            rows.append(row)
            groups[observation_group(f, observation)].append(row)
    return {"status": "complete", "sample_stride": stride, "sampled_frames": sampled,
            "replayed_observations": len(rows), "exclusions": dict(excluded),
            "metrics": summarize_rows(rows), "groups": {k: summarize_rows(v) for k, v in groups.items()},
            "interpretation": "Replay differences check reproducibility; small reprojection error cannot prove true depth/yaw or resolve planar ambiguity. No pose is fed back into control."}


def analyze(frames, max_offset_ms=25.0):
    offset_ns = int(max_offset_ms * 1e6)
    observations, feedback, timelines = defaultdict(list), defaultdict(dict), defaultdict(list)
    invalid_observations = 0
    for f in frames:
        run, epoch, t = f["run"], f.get("track_epoch"), f["frame_ns"]
        timelines[run].append((t, epoch))
        if f.get("frame_imu_valid") and f.get("tracker_converged") and f.get("target_present") and not f.get("observations_truncated"):
            counts = Counter(o.get("face_id") for o in f.get("observations", []))
            for o in f.get("observations", []):
                face = o.get("face_id")
                xyz, yaw = o.get("xyz_world_m"), o.get("body_yaw_rad")
                if not isinstance(face, int) or face < 0 or counts[face] != 1 or not vector(xyz) or not finite(yaw):
                    invalid_observations += 1
                    continue
                observations[(run, epoch, face)].append((t, xyz[:3] + [yaw]))
        # Convert the timestamped IMU-history pose back to the raw packet convention.
        # Do not pair the asynchronously updated raw atomics with an earlier timestamp.
        rt = f.get("feedback_pose_ns")
        wy, wp = f.get("feedback_world_yaw_rad"), f.get("feedback_world_pitch_rad")
        sy, sp = f.get("feedback_imu_yaw_scale"), f.get("feedback_imu_pitch_scale")
        oy, op = f.get("feedback_imu_yaw_offset_rad"), f.get("feedback_imu_pitch_offset_rad")
        if all(finite(x) for x in [rt, wy, wp, sy, sp, oy, op]) and sy != 0 and sp != 0:
            feedback[run][rt] = [(wy - oy) / sy, (wp - op) / sp]
    obs_series = {key: series(points) for key, points in observations.items()}
    angle_series = {key: (data[0], [[v[3]] for v in data[1]]) for key, data in obs_series.items()}
    fb_series = {run: series(points.items()) for run, points in feedback.items()}
    timeline_times = {run: [p[0] for p in points] for run, points in timelines.items()}
    excluded, residuals, tracking, tracking_excluded = Counter(), [], [], Counter()
    for f in frames:
        run, epoch, t = f["run"], f.get("track_epoch"), f["frame_ns"]
        tx_t = f.get("tx_write_complete_ns")
        tx = [f.get("tx_raw_yaw_rad"), f.get("tx_raw_pitch_rad")]
        if f.get("tx_ok") and f.get("tx_control") and finite(tx_t) and vector(tx, 2):
            pose, gap, reason = interpolate(fb_series.get(run, ([], [])), tx_t, offset_ns, angular=True)
            if reason:
                tracking_excluded[reason] += 1
            else:
                tracking.append({"run": run, "frame_seq": f["frame_seq"],
                                 "yaw_error_deg": math.degrees(math.remainder(tx[0] - pose[0], math.tau)),
                                 "pitch_error_deg": math.degrees(math.remainder(tx[1] - pose[1], math.tau)),
                                 "reference_bracket_ms": gap})
        if not f.get("native_prediction_valid"):
            excluded["invalid_prediction"] += 1
            continue
        if f.get("observations_truncated"):
            excluded["truncated_observations"] += 1
            continue
        if not f.get("native_prediction_trusted") or not f.get("frame_imu_valid"):
            excluded["untrusted_or_unsynced"] += 1
            continue
        if not f.get("native_iteration_converged"):
            excluded["iteration_not_converged"] += 1
            continue
        pt, face, predicted = f.get("native_prediction_ns"), f.get("native_face_id"), f.get("native_xyza")
        if not finite(pt) or pt <= t or not isinstance(face, int) or face < 0 or not vector(predicted, 4):
            excluded["invalid_prediction_metadata"] += 1
            continue
        times = timeline_times[run]
        if pt > times[-1]:
            excluded["run_ended_before_prediction"] += 1
            continue
        # Require uninterrupted identity up to and including the future bracket frame.
        first, last = bisect_left(times, t), bisect_left(times, pt)
        if any(e != epoch for _, e in timelines[run][first:last + 1]):
            excluded["track_epoch_changed"] += 1
            continue
        reference, gap, reason = interpolate(obs_series.get((run, epoch, face), ([], [])), pt, offset_ns)
        if reason:
            excluded["same_face_" + reason] += 1
            continue
        # Body angle needs circular interpolation; XYZ uses ordinary interpolation.
        angular_reference, _, _ = interpolate(
            angle_series[(run, epoch, face)], pt, offset_ns, angular=True)
        reference[3] = angular_reference[0]
        yaw_p, yaw_r = math.atan2(predicted[1], predicted[0]), math.atan2(reference[1], reference[0])
        pitch_p = math.atan2(predicted[2], math.hypot(*predicted[:2]))
        pitch_r = math.atan2(reference[2], math.hypot(*reference[:2]))
        delta = [predicted[i] - reference[i] for i in range(3)]
        residuals.append({"run": run, "frame_seq": f["frame_seq"], "track_epoch": epoch,
                          "face_id": face, "motion_mode": f.get("motion_mode"),
                          "rotation_sign": "positive" if (f.get("omega_rad_s") or 0) > 0 else "negative",
                          "horizon_ms": (pt - t) / 1e6, "reference_bracket_ms": gap,
                          "yaw_error_deg": math.degrees(math.remainder(yaw_p - yaw_r, math.tau)),
                          "pitch_error_deg": math.degrees(pitch_p - pitch_r),
                          "transverse_error_cm": 100 * (-math.sin(yaw_r) * delta[0] + math.cos(yaw_r) * delta[1]),
                          "xyz_error_cm": 100 * math.sqrt(sum(x * x for x in delta)),
                          "phase_error_deg": math.degrees(math.remainder(predicted[3] - reference[3], math.tau))})
    groups = defaultdict(list)
    for row in residuals:
        groups[f"{row['run']}/{row['motion_mode']}/{row['rotation_sign']}/face{row['face_id']}"].append(row)
    states = [f["state_at_frame"] for f in frames if vector(f.get("state_at_frame"), 11)]
    return {
        "frames": len(frames), "matched_predictions": len(residuals),
        "match_fraction_all_frames": len(residuals) / len(frames) if frames else 0,
        "prediction_exclusions": dict(excluded), "invalid_observations": invalid_observations,
        "prediction_error": summarize_residuals(residuals),
        "prediction_groups": {key: summarize_residuals(rows) for key, rows in groups.items()},
        "simultaneous_tracking_error": {field: stats([r[field] for r in tracking])
                                         for field in ["yaw_error_deg", "pitch_error_deg", "reference_bracket_ms"]},
        "tracking_exclusions": dict(tracking_excluded),
        "attribution": analyze_attribution(frames, residuals),
        "dual_geometry": analyze_dual_geometry(frames),
        "yaw_zero_shadow": analyze_yaw_zero_shadow(frames),
        "camera_policy": analyze_camera_policy(frames),
        "center_speed_mps": stats([math.hypot(s[1], s[3]) for s in states]),
        "debug_capture_us": stats([f.get("capture_us") for f in frames]),
        "frame_to_write_ms": stats([(f["tx_write_complete_ns"] - f["frame_ns"]) / 1e6
                                    for f in frames if f.get("tx_ok") and finite(f.get("tx_write_complete_ns"))]),
        "recorded_frame_interval_ms": stats([(right["frame_ns"] - left["frame_ns"]) / 1e6
                                              for left, right in zip(frames, frames[1:]) if left["run"] == right["run"]]),
        "observed_queue_dropped_max": max((f.get("queue_dropped_total", 0) for f in frames), default=0),
        "observed_sink_errors_max": max((f.get("sink_errors_total", 0) for f in frames), default=0),
        "residual_samples": residuals,
        "limitations": ["Visual PnP reference shares calibration and association errors; not independent truth or hit rate.",
                        "Face ids are tracker-relative; continuity checks cannot prove physical identity.",
                        "Only native Aimer predictions are scored; high-speed event predictions are separate.",
                        "Host image times are not verified exposure centers; no extrapolation is performed.",
                        "Tracking errors are simultaneous raw-angle residuals, not measured actuator latency.",
                        "Observed drop counters do not include unflushed data at forced process termination."]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--max-offset-ms", type=float, default=25.0)
    parser.add_argument("--start", help="Inclusive log wall time: HH:MM:SS or YYYY-MM-DD HH:MM:SS")
    parser.add_argument("--end", help="Inclusive log wall time (optional; normally use the actual log end)")
    parser.add_argument("--pnp-replay", action="store_true", help="Offline OpenCV replay of captured corners; never accesses hardware")
    parser.add_argument("--pnp-stride", type=int, default=10, help="Replay every Nth frame, default 10; no effect on other metrics")
    args = parser.parse_args()
    if not 0 < args.max_offset_ms <= 50:
        parser.error("--max-offset-ms must be within (0, 50]")
    if args.pnp_stride < 1:
        parser.error("--pnp-stride must be positive")
    from datetime import datetime
    for value in [args.start, args.end]:
        if value is not None:
            try:
                datetime.strptime(value, "%H:%M:%S" if len(value) == 8 else "%Y-%m-%d %H:%M:%S")
            except ValueError:
                parser.error("--start/--end must be HH:MM:SS or YYYY-MM-DD HH:MM:SS")
    all_frames, errors = read_frames(args.logs)
    contexts, conflicts = calibration_context(all_frames)
    def in_window(f):
        value = f.get("log_time")
        if (args.start or args.end) and not value:
            return False
        def compared(bound):
            return value[11:19] if len(bound) == 8 else value[:19]
        return (not args.start or compared(args.start) >= args.start) and (not args.end or compared(args.end) <= args.end)
    frames = [f for f in all_frames if in_window(f)]
    report = analyze(frames, args.max_offset_ms)
    report["parse_errors"] = errors
    report["input_logs"] = [str(p.resolve()) for p in args.logs]
    report["reference_max_offset_ms"] = args.max_offset_ms
    report["window"] = {"requested_start": args.start, "requested_end": args.end,
                        "first_frame": frames[0].get("log_time") if frames else None,
                        "last_frame": frames[-1].get("log_time") if frames else None}
    if args.pnp_replay:
        report["pnp_replay"] = analyze_pnp_replay(frames, contexts, conflicts, args.pnp_stride)
    if args.output:
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    # Keep console compact; per-sample evidence is in the optional output file.
    compact = {k: v for k, v in report.items() if k not in ["residual_samples", "prediction_groups", "attribution", "pnp_replay", "dual_geometry"]}
    compact["dual_geometry"] = {k: v for k, v in report["dual_geometry"].items()
                               if k not in ["groups", "view_angle_updates", "pair_samples"]}
    compact["attribution"] = {k: v for k, v in report["attribution"].items() if k not in ["current_groups", "future_groups"]}
    compact["yaw_zero_shadow"] = {k: v for k, v in report["yaw_zero_shadow"].items() if k != "samples"}
    if "pnp_replay" in report:
        compact["pnp_replay"] = {k: v for k, v in report["pnp_replay"].items() if k != "groups"}
    print(json.dumps(compact,
                     ensure_ascii=False, indent=2))
    if not frames:
        parser.exit(2, "No prediction_frame records: use the new debug build and enable diagnostics.\n")


if __name__ == "__main__":
    main()
