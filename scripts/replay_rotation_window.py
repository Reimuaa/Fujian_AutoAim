#!/usr/bin/env python3
"""Causal, offline comparison of rotation-window decisions in SHOT_DEBUG logs.

Never imports device drivers or sends commands. Recorded tracking, observations,
and gates are frozen: this is a decision replay, not a closed-loop simulation or
a prediction of actual projectile counts. The extra cooldown supplements (does
not replace) recorded command_rearm gates and models only a minimum interval.
"""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import statistics


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def phase_relative_to_center(row):
    if not row or not row.get("native_prediction_valid"):
        return None
    if row.get("native_geometry_source") != "whole_ekf_rigid":
        return None
    state = row.get("whole_ekf_state_at_frame")
    point = row.get("native_xyza")
    prediction_ns = row.get("native_prediction_ns")
    if not state or len(state) != 11 or not point or len(point) != 4:
        return None
    values = [*state, *point, prediction_ns, row.get("frame_ns")]
    if not all(finite(value) for value in values):
        return None
    dt = (prediction_ns - row["frame_ns"]) / 1e9
    if dt < 0:
        return None
    center_x = state[0] + state[1] * dt
    center_y = state[2] + state[3] * dt
    return math.remainder(point[3] - math.atan2(center_y, center_x), 2 * math.pi)


def exiting_next_step(previous, current, coming_deg, leaving_deg, margin_deg=0):
    """Only current/past data; mirror the exit edge for negative rotation.

    As in RotationFireWindow's error extrapolation, use one observed frame step.
    No future log row, future invalidity, or actual outcome is consulted here.
    """
    if not previous or current.get("armor_count") != 4:
        return False
    keys = ("track_epoch", "target_name", "target_type", "armor_count", "native_face_id")
    if any(current.get(key) != previous.get(key) for key in keys):
        return False
    if current.get("native_face_id", -1) < 0 or current.get("track_epoch_reset"):
        return False
    if current["frame_seq"] != previous["frame_seq"] + 1:
        return False
    if current["frame_ns"] <= previous["frame_ns"]:
        return False
    phase = phase_relative_to_center(current)
    old_phase = phase_relative_to_center(previous)
    if phase is None or old_phase is None:
        return False
    omega = current["whole_ekf_state_at_frame"][7]
    old_omega = previous["whole_ekf_state_at_frame"][7]
    if omega * old_omega <= 0:
        return False
    # Match Aimer's existing degree conversion, not a new selection boundary.
    coming, leaving = coming_deg / 57.3, leaving_deg / 57.3
    step = math.remainder(phase - old_phase, 2 * math.pi)
    margin = math.radians(margin_deg)
    if omega > 0:
        return (-coming <= phase < leaving and step > 0 and
                phase + step >= leaving - margin)
    return (-leaving < phase <= coming and step < 0 and
            phase + step <= -leaving + margin)


def eligible(row):
    return (row.get("fire_branch") == "vehicle_prediction" and
            bool(row.get("fire_gates")) and all(row["fire_gates"].values()))


def read_frames(path):
    rows = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            marker = "[SHOT_DEBUG] "
            if marker not in line:
                continue
            row = json.loads(line.split(marker, 1)[1])
            if row.get("kind") == "prediction_frame":
                row["log_line"] = line_number
                row["wall_time"] = line[1:24]
                rows.append(row)
    if not rows:
        raise ValueError(f"No prediction frames: {path}")
    return rows


def candidate_groups(rows):
    groups, group = [], []
    for row in rows + [None]:
        same = (row is not None and eligible(row) and group and
                row["frame_seq"] == group[-1]["frame_seq"] + 1 and
                row["track_epoch"] == group[-1]["track_epoch"] and
                row["native_face_id"] == group[-1]["native_face_id"])
        if group and not same:
            groups.append(group)
            group = []
        if row is not None and eligible(row):
            group.append(row)
    return groups


def decide(rows, coming_deg=60, leaving_deg=20, margin_deg=0, cooldown_ms=150):
    selected, previous, last_ns = [], None, -math.inf
    for row in rows:
        boundary = exiting_next_step(previous, row, coming_deg, leaving_deg, margin_deg)
        better = row["rotation_fire_window"]["current_frame_is_better"]
        now = row["aimer_call_end_ns"]
        if eligible(row) and (better or boundary) and now - last_ns >= cooldown_ms * 1e6:
            selected.append(row)
            last_ns = now
        previous = row
    return selected


def describe(row):
    return {"frame_seq": row["frame_seq"], "time": row["wall_time"],
            "line": row["log_line"], "face": row["native_face_id"],
            "error_deg": math.degrees(row["gate_error_rad"])}


def analyze(path, coming_deg, leaving_deg, margin_deg, cooldown_ms):
    rows = read_frames(path)
    baseline = [row for row in rows if eligible(row) and
                row["rotation_fire_window"]["current_frame_is_better"]]
    actual = [row for row in rows if row["fire_branch"] == "vehicle_prediction" and
              row["request_shoot"]]
    # Establish that our reading of the existing rule exactly matches the log.
    assert [r["frame_seq"] for r in baseline] == [r["frame_seq"] for r in actual]
    groups = candidate_groups(rows)
    selected = decide(rows, coming_deg, leaving_deg, margin_deg, cooldown_ms)
    new_ids = {row["frame_seq"] for row in selected}
    old_ids = {row["frame_seq"] for row in baseline}
    recovered, preserved, remaining, shifts = [], [], [], []
    duplicates = 0
    for group in groups:
        old = [row for row in group if row["frame_seq"] in old_ids]
        new = [row for row in group if row["frame_seq"] in new_ids]
        duplicates += max(0, len(new) - 1)
        if old and new:
            preserved.append(describe(new[0]))
            shifts.append((old[0]["aimer_call_end_ns"] - new[0]["aimer_call_end_ns"]) / 1e6)
        elif new:
            recovered.append(describe(new[0]))
        elif not old:
            remaining.append(describe(group[-1]))
    intervals = [(b["aimer_call_end_ns"] - a["aimer_call_end_ns"]) / 1e6
                 for a, b in zip(selected, selected[1:])]
    assert all(eligible(row) for row in selected)
    assert all(abs(row["gate_error_rad"]) <= row["gate_tolerance_rad"] for row in selected)
    assert all(interval >= cooldown_ms for interval in intervals)
    assert duplicates == 0
    return {
        "log": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "frames": len(rows), "baseline_reproduced_exactly": True,
        "candidate_windows": len(groups), "baseline_requests": len(baseline),
        "replay_requests": len(selected), "preserved_baseline_windows": len(preserved),
        "recovered_windows": len(recovered), "remaining_missed_windows": len(remaining),
        "duplicate_requests_in_candidate_window": duplicates,
        "all_existing_gates_and_error_tolerance_passed": True,
        "minimum_interval_ms": min(intervals) if intervals else None,
        "maximum_abs_error_deg": max(abs(math.degrees(r["gate_error_rad"])) for r in selected),
        "preserved_window_advance_ms": {"min": min(shifts) if shifts else None,
                                        "median": statistics.median(shifts) if shifts else None,
                                        "max": max(shifts) if shifts else None},
        "baseline_by_face": dict(collections.Counter(r["native_face_id"] for r in baseline)),
        "replay_by_face": dict(collections.Counter(r["native_face_id"] for r in selected)),
        "recovered": recovered, "remaining": remaining,
        "selected": [describe(row) for row in selected],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--coming-deg", type=float, default=60)
    parser.add_argument("--leaving-deg", type=float, default=20)
    parser.add_argument("--margin-deg", type=float, default=0)
    parser.add_argument("--cooldown-ms", type=float, default=150)
    args = parser.parse_args()
    if not (0 <= args.margin_deg < args.leaving_deg < args.coming_deg < 90):
        parser.error("Require 0 <= margin < leaving < coming < 90")
    if args.cooldown_ms <= 0:
        parser.error("cooldown-ms must be positive")
    output = args.output.resolve()
    log_root = Path(__file__).resolve().parents[1] / "logs"
    if not output.is_relative_to(log_root.resolve()) or output in [p.resolve() for p in args.logs]:
        parser.error("Report must be inside logs/ and must not overwrite an input log")
    results = [analyze(p, args.coming_deg, args.leaving_deg, args.margin_deg,
                       args.cooldown_ms) for p in args.logs]
    report = {
        "method": "causal_current_and_previous_frame_boundary_check",
        "parameters": {"coming_deg": args.coming_deg, "leaving_deg": args.leaving_deg,
                       "margin_deg": args.margin_deg, "minimum_interval_ms": args.cooldown_ms},
        "limitations": ["Recorded tracking, gimbal trajectory, and all gates are frozen.",
                        "Recorded command_rearm gates are retained; an extra minimum interval is imposed.",
                        "No device I/O, runtime code change, physical execution or accuracy prediction.",
                        "All results are vehicle_prediction only; other branches are excluded.",
                        "Next-step estimation cannot guarantee future detections or selected-face validity."],
        "results": results,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    for result in results:
        print(json.dumps({k: v for k, v in result.items()
                          if k not in {"recovered", "remaining", "selected"}}, ensure_ascii=True))


if __name__ == "__main__":
    main()
