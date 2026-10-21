#!/usr/bin/env python3
"""Local-only review of a rotating-target log. No device I/O or config writes.

The bounded per-face correction below is an OFFLINE CANDIDATE, not a controller.
Future observations grade predictions; they may enter the candidate's history
only after the prediction horizon plus the complete reference bracket elapsed.
One closed visibility interval contributes one sample, rather than treating
correlated adjacent camera frames as independent calibration cycles.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics

import analyze_prediction_debug as review


class MaturedFaceReview:
    def __init__(self, cap_deg=0.3, min_cycles=3, age_s=8.0, gap_s=0.25):
        if not (0 < cap_deg <= 1 and min_cycles >= 3 and age_s > gap_s > 0):
            raise ValueError("Invalid candidate-review settings")
        self.cap_deg = cap_deg
        self.min_cycles = min_cycles
        self.age_ns = round(age_s * 1e9)
        self.gap_ns = round(gap_s * 1e9)
        self.groups = {}
        self.cycles = defaultdict(list)

    def close(self, key):
        last_ns, values = self.groups.pop(key)
        self.cycles[key].append((last_ns, statistics.median(values)))

    def observe_matured(self, key, available_ns, error_deg):
        if not review.finite(error_deg):
            return
        if key in self.groups:
            last_ns, _ = self.groups[key]
            if available_ns < last_ns:
                raise ValueError("Matured residuals must arrive in time order")
            if available_ns - last_ns > self.gap_ns:
                self.close(key)
        if key not in self.groups:
            self.groups[key] = (available_ns, [])
        _, values = self.groups[key]
        values.append(error_deg)
        self.groups[key] = (available_ns, values)

    def advance(self, now_ns):
        for key, (last_ns, _) in list(self.groups.items()):
            if now_ns - last_ns > self.gap_ns:
                self.close(key)
        for key in list(self.cycles):
            self.cycles[key] = [(t, v) for t, v in self.cycles[key]
                                if 0 <= now_ns - t <= self.age_ns]

    def correction(self, key):
        samples = self.cycles.get(key, [])
        if len(samples) < self.min_cycles:
            return 0.0
        bias = statistics.median(v for _, v in samples)
        return -max(-self.cap_deg, min(self.cap_deg, bias))


def frame_key(frame):
    return frame["run"], frame["frame_seq"]


def face_key(frame, face):
    omega = frame.get("omega_rad_s") or 0
    return frame["run"], frame.get("track_epoch"), face, 1 if omega > 0 else -1


def causal_candidate(frames, residuals, max_offset_ms=25.0, cap_deg=0.3):
    """Replay a past-only candidate; freeze learning at the time midpoint too.

    Predictions without a future match remain in the query stream. Their
    absence from the eventual grading set must not decide online state changes.
    Midpoint grading uses fixed first-half coefficients, with no second-half
    learning. Frozen-log results do not simulate new requests or a new trajectory.
    """
    if len({f["run"] for f in frames}) > 1:
        raise ValueError("Review one recorded run at a time")
    if not frames:
        return {"samples": [], "corrections": {}, "midpoint_ns": None}
    lookup = {frame_key(f): f for f in frames}
    ready = sorted(((lookup[(r["run"], r["frame_seq"])]["native_prediction_ns"] +
                     round(max_offset_ms * 1e6), r) for r in residuals), key=lambda q: q[0])
    monitor = MaturedFaceReview(cap_deg=cap_deg)
    midpoint_ns = (frames[0]["frame_ns"] + frames[-1]["frame_ns"]) // 2
    frozen, frozen_ready, cursor = {}, False, 0
    corrections = {}
    for f in frames:
        now_ns = f["frame_ns"]
        # Never learn from a reference that has not arrived by the freeze time.
        cutoff = midpoint_ns if not frozen_ready and now_ns >= midpoint_ns else now_ns
        while cursor < len(ready) and ready[cursor][0] <= cutoff:
            available_ns, r = ready[cursor]
            cursor += 1
            origin = lookup[(r["run"], r["frame_seq"])]
            if (origin.get("motion_mode") == "vehicle" and
                    abs(origin.get("omega_rad_s") or 0) >= 3.2):
                monitor.observe_matured(face_key(origin, r["face_id"]),
                                         available_ns, r["yaw_error_deg"])
        monitor.advance(cutoff)
        if not frozen_ready and now_ns >= midpoint_ns:
            frozen = {k: monitor.correction(k) for k in monitor.cycles}
            frozen_ready = True
        # Queries between cutoff and now may use new evidence online, whereas
        # the first-half-only benchmark remains fixed.
        while cursor < len(ready) and ready[cursor][0] <= now_ns:
            available_ns, r = ready[cursor]
            cursor += 1
            origin = lookup[(r["run"], r["frame_seq"])]
            if (origin.get("motion_mode") == "vehicle" and
                    abs(origin.get("omega_rad_s") or 0) >= 3.2):
                monitor.observe_matured(face_key(origin, r["face_id"]),
                                         available_ns, r["yaw_error_deg"])
        monitor.advance(now_ns)
        key = face_key(f, f.get("native_face_id"))
        eligible = f.get("motion_mode") == "vehicle" and abs(f.get("omega_rad_s") or 0) >= 3.2
        corrections[frame_key(f)] = {
            "online_deg": monitor.correction(key) if eligible else 0.0,
            "frozen_deg": frozen.get(key, 0.0) if eligible else 0.0,
        }
    samples = []
    for r in residuals:
        f = lookup[(r["run"], r["frame_seq"])]
        c = corrections[frame_key(f)]
        samples.append({"run": r["run"], "frame_seq": r["frame_seq"],
                        "face_id": r["face_id"], "request": bool(f.get("request_shoot")),
                        "second_half": f["frame_ns"] >= midpoint_ns,
                        "baseline_deg": r["yaw_error_deg"],
                        "candidate_deg": r["yaw_error_deg"] + c["online_deg"],
                        "first_half_frozen_deg": r["yaw_error_deg"] + c["frozen_deg"]
                        if f["frame_ns"] >= midpoint_ns else None})
    return {"samples": samples, "corrections": corrections, "midpoint_ns": midpoint_ns}


def candidate_summary(rows):
    return {field: review.stats([r[field] for r in rows]) for field in
            ("baseline_deg", "candidate_deg", "first_half_frozen_deg")}


def analyze_window(path, start, end, max_offset_ms=25.0):
    all_frames, parse_errors = review.read_frames([path])
    frames = [f for f in all_frames if f.get("log_time") and
              start <= f["log_time"][11:] < end]
    if not frames:
        raise ValueError("No prediction frames in requested [start, end) window")
    base = review.analyze(frames, max_offset_ms)
    lookup = {frame_key(f): f for f in frames}
    residuals = base["residual_samples"]
    sources = {frame_key(f) for f in frames}
    events, request_time = Counter(), []
    for line in path.open(encoding="utf-8", errors="replace"):
        if "[SHOT_DEBUG] " not in line:
            continue
        try:
            item = json.loads(line.split("[SHOT_DEBUG] ", 1)[1])
        except ValueError:
            continue
        if item.get("kind") not in ("first_high", "first_low", "request_snapshot"):
            continue
        if (item.get("run"), item.get("frame_seq")) not in sources:
            continue
        if item.get("kind") in ("first_high", "first_low", "request_snapshot"):
            events[item["kind"]] += 1
        if item.get("kind") == "first_high" and item.get("tx_ok"):
            request_time.append((item["tx_write_complete_ns"] - item["frame_ns"]) / 1e6)
    candidate = causal_candidate(frames, residuals, max_offset_ms)
    per_face = {}
    for face in range(4):
        matched = [r for r in residuals if r["face_id"] == face]
        per_face[str(face)] = {
            "yaw_error_deg": review.stats([r["yaw_error_deg"] for r in matched]),
            "phase_error_deg": review.stats([r["phase_error_deg"] for r in matched]),
            "requests": sum(bool(f.get("request_shoot")) for f in frames
                            if f.get("native_face_id") == face)}
    scored_requests = [r for r in residuals if lookup[(r["run"], r["frame_seq"])].get("request_shoot")]
    active = [f for f in frames if f.get("work_zone_target_state") == "tracking"]
    rows = candidate["samples"]
    return {
        "input": str(path.resolve()), "input_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "window": {"start_inclusive": start, "end_exclusive": end,
                   "first": frames[0]["log_time"], "last": frames[-1]["log_time"]},
        "frames": len(frames), "parse_errors": parse_errors,
        "recorded_delay_s": sorted({f["native_delay_s"] for f in frames if review.finite(f.get("native_delay_s"))}),
        "recorded_gate_deg": sorted({round(math.degrees(f["gate_tolerance_rad"]), 6) for f in frames
                                     if review.finite(f.get("gate_tolerance_rad"))}),
        "motion_modes": dict(Counter(f.get("motion_mode") for f in frames)),
        "fire_branches": dict(Counter(f.get("fire_branch") for f in frames)),
        "active_omega_rad_s": review.stats([f.get("omega_rad_s") for f in active]),
        "prediction_error": base["prediction_error"],
        "prediction_exclusions": base["prediction_exclusions"],
        "matched_predictions": len(residuals), "per_face": per_face,
        "requests": {"issued": sum(bool(f.get("request_shoot")) for f in frames),
                     "events": dict(events), "matched_predictions": len(scored_requests),
                     "visual_yaw_error_deg": review.stats([r["yaw_error_deg"] for r in scored_requests]),
                     "image_to_confirmed_high_ms": review.stats(request_time)},
        "frame_interval_ms": base["recorded_frame_interval_ms"],
        "frame_to_write_ms": base["frame_to_write_ms"],
        "current_geometry": {k: base["attribution"][k] for k in
                             ("current_prior_vs_pnp", "current_posterior_vs_pnp", "dual_face_same_frame")},
        "candidate": {"runtime_enabled": False, "description": "Past matured per-face residual, median of >=3 closed visibility intervals in last 8s, correction capped at +/-0.3deg; high-speed vehicle only",
                      "all_scored": candidate_summary(rows),
                      "second_half": candidate_summary([r for r in rows if r["second_half"]]),
                      "same_original_requests": candidate_summary([r for r in rows if r["request"]]),
                      "midpoint_ns": candidate["midpoint_ns"], "samples": rows},
        "limitations": [
            "只有一轮顺时针高速；本日志没有逆时针或低速验收数据。",
            "未来参考为同epoch同面已接受PnP，插值两侧各不超过25ms；共享标定和关联误差，不是独立真值或命中率。",
            "分析区间外的参考已排除；没有匹配到的预测和请求不视为正确。",
            "面ID来自Tracker；尚不能独立区分关联错误、PnP偏差和真实几何参数误差。",
            "330ms是配置的预测补偿，日志没有独立测得的真实离膛时间。",
            "候选只评价冻结轨迹下原预测/请求时刻，不代表新请求数、实弹出弹数或闭环命中率。",
            "候选没有接入运行程序；不同面反向偏差可能需要修正几何，不能直接把视觉残差当物理弹着补偿。",
        ]}


def markdown(report):
    p = report["prediction_error"]
    requests = report["requests"]
    lines = ["# 高速旋转专项检查", "", f"输入：`{Path(report['input']).name}`；SHA-256 `{report['input_sha256']}`。",
             f"区间：{report['window']['start_inclusive']}（含）～{report['window']['end_exclusive']}（不含）；{report['frames']}帧。",
             "", "本次只在本机做日志分析和离线候选回放，没有连接小电脑，没有修改运行算法、正式配置或已有本地改动。",
             "", "## 直接证据", "",
             f"- 日志实际补偿 {report['recorded_delay_s']} s，门限 {report['recorded_gate_deg']}°；高速走vehicle，未走事件PLL。",
             f"- {report['matched_predictions']}个同epoch同板预测可评分，水平绝对误差P95 {p['yaw_error_deg']['abs_p95']:.3f}°，有符号均值 {p['yaw_error_deg']['mean']:+.3f}°。",
             f"- 预测视界中位数 {p['horizon_ms']['median']:.1f}ms；图像到写包中位数 {report['frame_to_write_ms']['median']:.1f}ms。",
             f"- {requests['issued']}次软件请求；{requests['events'].get('first_high',0)}个确认高电平包；其中{requests['matched_predictions']}次有同板未来视觉参考。不是实际出弹或命中计数。",
             "- 0.3°门限检查的是模型预测点与瞄准线的距离，不能保证模型预测点本身只有0.3°误差。",
             "", "| 跟踪面ID | 可评分预测 | 水平均值 | 水平绝对P95 | 请求数 |", "|---|---:|---:|---:|---:|"]
    for face, data in report["per_face"].items():
        e = data["yaw_error_deg"]
        lines.append(f"| {face} | {e['n']} | {e['mean']:+.3f}° | {e['abs_p95']:.3f}° | {data['requests']} |")
    lines += ["", "这些ID不等于经过独立验证的物理装甲身份，不能据此屏蔽某一块装甲。方向相反的偏差说明统一调延迟或继续收窄门限不能同时修正所有面。",
              "", "## 改进顺序", "",
              f"1. 优先核对分面PnP/安装几何与整车中心、半径、相位的一致性。当前先验/后验对PnP的水平残差P95分别为{report['current_geometry']['current_prior_vs_pnp']['yaw_error_deg']['abs_p95']:.3f}°/{report['current_geometry']['current_posterior_vs_pnp']['yaw_error_deg']['abs_p95']:.3f}°；这不是物理精度的先后优劣证明，提示应核查观测和几何的一致性，不直接加大角速度滤波。",
              "2. 在高速vehicle分支增加只使用已经到达的未来对照的分面残差监测。目标重建、身份不连续、换向或缺少参考时撤销学习；低速保持当前链路。先作为影子诊断验证，避免直接闭环补偿PnP误差。",
              "3. 物理发弹延迟和实际弹速单独测量；不能把330ms或电流事件当实测离膛时间。已有事件PLL试验已回退，本轮不重新启用。",
              "4. 补顺/逆时针高速、低速回归和独立实弹落点验收，再决定是否接入补偿。",
              "", "## 有限候选回放", "",
              "候选仅使用当时已经成熟的旧预测残差；按相隔超过250ms的对照片段分别汇总，每段只计一次样本，至少3段后才可用，最大修正±0.3°。后半段固定参数结果只使用前半段已成熟的数据。以下不等于新控制闭环或命中率。",
              "", "| 样本 | 原P95 | 持续因果候选P95 | 前半段固定参数P95 |", "|---|---:|---:|---:|"]
    for name, label in [("all_scored", "全部可评分预测"), ("second_half", "后半段"), ("same_original_requests", "原请求时刻")]:
        d = report["candidate"][name]
        frozen = f"{d['first_half_frozen_deg']['abs_p95']:.3f}°" if name == "second_half" else "仅后半段有验证意义"
        lines.append(f"| {label} | {d['baseline_deg']['abs_p95']:.3f}° | {d['candidate_deg']['abs_p95']:.3f}° | {frozen} |")
    lines += ["", "候选改善支持继续调查分面系统误差，但不能独立证明PnP偏差、几何参数或关联哪一项是物理根因；正式控制没有接入这个候选。",
              "", "## 限制", ""] + ["- " + s for s in report["limitations"]]
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--start", default="23:18:15")
    parser.add_argument("--end", default="23:19:00")
    parser.add_argument("--output", type=Path, default=Path("logs/high_speed_review.json"))
    args = parser.parse_args()
    from datetime import datetime
    for value in (args.start, args.end):
        datetime.strptime(value, "%H:%M:%S")
    if args.start >= args.end:
        parser.error("Use an increasing same-day [start, end) window")
    report = analyze_window(args.log, args.start, args.end)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    md_path = args.output.with_suffix(".md")
    md_path.write_text(markdown(report), encoding="utf-8")
    print(json.dumps({"frames": report["frames"], "matched": report["matched_predictions"],
                      "requests": report["requests"]["issued"], "per_face": report["per_face"],
                      "candidate_second_half": report["candidate"]["second_half"],
                      "report": str(md_path)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
