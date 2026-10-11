#!/usr/bin/env python3
"""Linux offline test of actual Aimer/Target code; no camera, serial, motor or standard process."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import tempfile
from collections import Counter

HARNESS = r'''
#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "tasks/auto_aim/aimer.hpp"
#include "tools/trajectory.hpp"
#include "tools/math_tools.hpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
  if (argc == 4) {
    std::ifstream input(argv[2]); std::ofstream output(argv[3]);
    assert(input && output);
    auto_aim::Aimer aimer(argv[1]);
    std::string line;
    while (std::getline(input, line)) {
      auto item = nlohmann::json::parse(line);
      const auto time = steady_clock::time_point(nanoseconds(item["frame_ns"].get<int64_t>()));
      auto_aim::Armor armor(11, .9f, cv::Rect(0,0,100,40), {{0,0},{100,0},{100,40},{0,40}});
      armor.xyz_in_world = {3.3, 0., .05}; armor.ypr_in_world = {0.,0.,0.};
      auto_aim::Target target(armor, time, .3, 4, Eigen::VectorXd::Ones(11));
      auto & state = const_cast<tools::ExtendedKalmanFilter &>(target.ekf()).x;
      for (int i=0;i<11;++i) state[i]=item["whole_state"][i].get<double>();
      target.jumped = true;
      auto_aim::AimPredictionTrace trace;
      // Reproduce the recorded base time without using the wall clock. Target
      // still starts at frame_ns; the Aimer timestamp simply selects that base.
      const auto base = item["base_ns"].get<int64_t>();
      auto command=aimer.aim({target}, steady_clock::time_point(nanoseconds(base-90000000)),
        item["speed"].get<double>(), false, &trace, auto_aim::AimGeometry::whole_ekf);
      assert(trace.whole_ekf_geometry && trace.base_prediction_ns==base);
      item["valid"] = command.control && trace.valid;
      item["converged"] = trace.converged;
      item["face"] = trace.face_id; item["prediction_ns"] = trace.prediction_ns;
      item["command_yaw"] = command.yaw; item["command_pitch"] = command.pitch;
      item["iterations"] = trace.iterations;
      item["xyza"] = nlohmann::json::array();
      if (trace.valid) for(int i=0;i<4;++i) item["xyza"].push_back(aimer.debug_aim_point.xyza[i]);
      output << item.dump() << '\n';
    }
    return 0;
  }
  const auto stamp = steady_clock::time_point(seconds(10));
  int checked = 0, valid = 0;
  for (double phase : {-2.7, -1.1, -.2, 0., .3, 1.2, 2.8})
  for (double omega : {-3., -1.5, 0., 1.5, 3.})
  for (double speed : {0., 28., 38.}) {
    auto_aim::Armor armor(11, .9f, cv::Rect(0,0,100,40),
      {{0,0},{100,0},{100,40},{0,40}});
    armor.xyz_in_world = {3.7, .1, .05}; armor.ypr_in_world = {phase,0,0};
    auto_aim::Target target(armor, stamp, .3, 4, Eigen::VectorXd::Ones(11));
    auto & state = const_cast<tools::ExtendedKalmanFilter &>(target.ekf()).x;
    state[7] = omega;
    target.jumped = true;
    auto_aim::Aimer compact(argv[1]), debug(argv[1]);
    auto_aim::AimPredictionTrace trace;
    auto a = compact.aim({target}, stamp, speed, false);
    auto b = debug.aim({target}, stamp, speed, false, &trace);
    assert(a.control==b.control && a.shoot==b.shoot && a.yaw==b.yaw && a.pitch==b.pitch);
    ++checked;
    if (!b.control) { assert(!trace.valid); continue; }
    ++valid; assert(trace.valid && trace.face_id>=0 && trace.face_id<4);
    assert(trace.speed_mps==(speed==28. ? 28. : 23.));
    assert(trace.base_prediction_ns==duration_cast<nanoseconds>(stamp.time_since_epoch()).count()+90000000);
    assert(trace.prediction_ns-trace.base_prediction_ns==
      static_cast<int64_t>(trace.flight_time_used_s*1e6)*1000);
    auto future=target; future.predict(steady_clock::time_point(nanoseconds(trace.prediction_ns)));
    const auto xyza=future.armor_xyza_list(std::abs(omega)<=.8)[trace.face_id];
    assert((xyza-debug.debug_aim_point.xyza).norm()<1e-10);
    assert(trace.iterations>=1 && trace.iterations<=10);
    if(trace.converged) assert(std::abs(trace.flight_time_used_s-trace.flight_time_s)<.001);
    auto empty=debug.aim({}, stamp, speed, false, &trace);
    assert(!empty.control && !trace.valid && trace.face_id==-1 && trace.prediction_ns==0);
  }
  assert(checked==105 && valid>30);
  std::cout << "PASS " << checked << " Aimer cases: trace on/off commands equal, exact prediction time/face, speed fallback, empty reset; valid=" << valid << "\n";
  // Train a real ESKF, then deliberately separate the two estimates. This
  // catches accidentally retaining the ESKF phase/rate in the new geometry.
  for (double direction : {-1., 1.}) {
    auto_aim::Armor armor(11, .9f, cv::Rect(0,0,100,40), {{0,0},{100,0},{100,40},{0,40}});
    armor.xyz_in_world={3.3,0.,.05}; armor.ypr_in_world={0.,0.,0.};
    auto_aim::Target trained(armor, stamp, .3, 4, Eigen::VectorXd::Ones(11));
    auto time=stamp;
    for(int i=1;i<=200;++i) {
      time=stamp+milliseconds(10*i); const double phase=direction*1.5*.01*i;
      armor.xyz_in_world={3.6-.3*std::cos(phase),-.3*std::sin(phase),.05};
      armor.ypr_in_world={phase,0.,0.};
      armor.ypd_in_world=tools::xyz2ypd(armor.xyz_in_world);
      trained.predict(time); trained.update(armor,0);
    }
    assert(trained.yaw_eskf_trusted());
    for(double phase : {-2.7,-1.1,-.2,0.,.3,1.2,2.8}) {
      auto target=trained;
      auto & state=const_cast<tools::ExtendedKalmanFilter &>(target.ekf()).x;
      state << 3.6,.1,.1,-.05,.05,0.,phase,direction*1.3,.28,.04,.03;
      target.jumped=true;
      const auto combined_before=target.ekf_x();
      auto_aim::Aimer compact(argv[1]), debug(argv[1]); auto_aim::AimPredictionTrace trace;
      const auto a=compact.aim({target},time,28.,false,nullptr,auto_aim::AimGeometry::whole_ekf);
      const auto b=debug.aim({target},time,28.,false,&trace,auto_aim::AimGeometry::whole_ekf);
      assert(a.control==b.control && a.yaw==b.yaw && a.pitch==b.pitch && trace.whole_ekf_geometry);
      assert((target.ekf_x()-combined_before).norm()==0.); // No mutation of association state.
      if(b.control) {
        auto future=target; future.predict(steady_clock::time_point(nanoseconds(trace.prediction_ns)));
        const auto expected=future.whole_ekf_armor_xyza_list()[trace.face_id];
        assert((expected-debug.debug_aim_point.xyza).norm()<1e-10);
        assert((future.armor_xyza_list()[trace.face_id]-expected).norm()>.01);
      }
      for(int excluded : {0,1,2}) {
        auto other=target;
        if(excluded==0) other.name=auto_aim::ArmorName::outpost;
        if(excluded==1) { // Untrusted stationary initialization preserves direct-plate selection.
          other=auto_aim::Target(armor,time,.3,4,Eigen::VectorXd::Ones(11));
        }
        if(excluded==2) other=auto_aim::Target(armor,time,.3,3,Eigen::VectorXd::Ones(11));
        auto_aim::Aimer legacy(argv[1]), requested(argv[1]);
        const auto old=legacy.aim({other},time,28.,false);
        const auto unchanged=requested.aim({other},time,28.,false,&trace,auto_aim::AimGeometry::whole_ekf);
        assert(!trace.whole_ekf_geometry && old.control==unchanged.control &&
          old.yaw==unchanged.yaw && old.pitch==unchanged.pitch);
      }
    }
  }
  std::cout << "PASS trusted dual-filter geometry: both directions, ballistic iterations, compact/debug equivalence, stationary/outpost/three-face fallback\n";
}
'''


def replay_log(root, binary, config, temp, path):
    """Run actual C++ selection/ballistics from recovered whole-EKF snapshots.

    Association, IMU, base prediction time and measured speed stay recorded.
    Future matching follows the existing same-face/no-extrapolation evaluator.
    This is not a tracker replay, actuator simulation or projectile hit test.
    """
    spec = importlib.util.spec_from_file_location("prediction_analysis", root / "scripts/analyze_prediction_debug.py")
    analysis = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(analysis)
    frames, errors = analysis.read_frames([path])
    assert errors == 0, errors
    by_key = {(f["run"], f["frame_seq"]): f for f in frames}
    excluded, fixtures = Counter(), []
    for frame, following in zip(frames, frames[1:]):
        if (frame.get("motion_mode") != "vehicle" or not frame.get("native_prediction_trusted")
                or not frame.get("native_iteration_converged") or frame.get("armor_count") != 4
                or frame.get("tracker_initialized") or frame.get("track_epoch_reset")):
            excluded["not_stable_vehicle"] += 1
            continue
        dt = (following["frame_ns"] - frame["frame_ns"]) / 1e9
        prior, mixed = following.get("state_before_update"), frame.get("state_at_frame")
        if (frame["run"] != following["run"] or frame.get("track_epoch") != following.get("track_epoch")
                or not 0 < dt <= .04 or not analysis.vector(prior, 11) or not analysis.vector(mixed, 11)
                or following.get("tracker_initialized") or following.get("track_epoch_reset")):
            excluded["cannot_recover_raw_posterior"] += 1
            continue
        state = list(prior)
        for position, velocity in [(0, 1), (2, 3), (4, 5), (6, 7)]:
            state[position] -= state[velocity] * dt
        state[6] = math.remainder(state[6], math.tau)
        if max(abs(state[i] - mixed[i]) for i in [0, 1, 2, 3, 4, 5, 8, 9, 10]) > 1e-7:
            excluded["state_continuity_mismatch"] += 1
            continue
        if not (.05 < state[8] < .5 and .05 < state[8] + state[9] < .5 and abs(state[7]) > .8):
            excluded["raw_state_outside_geometry_path"] += 1
            continue
        base, speed = frame.get("native_base_prediction_ns"), frame.get("speed_mps")
        if not analysis.finite(base) or not analysis.finite(speed):
            excluded["missing_recorded_timing_or_speed"] += 1
            continue
        fixtures.append({"run": frame["run"], "frame_seq": frame["frame_seq"],
                         "frame_ns": frame["frame_ns"], "base_ns": base,
                         "whole_state": state, "speed": speed})
    input_path, output_path = temp / "replay_input.jsonl", temp / "replay_output.jsonl"
    input_path.write_text("".join(json.dumps(item) + "\n" for item in fixtures), encoding="utf-8")
    subprocess.run([str(binary), str(config), str(input_path), str(output_path)], check=True, timeout=120, cwd=root)
    original, replayed = [], []
    changed_face = invalid = nonconverged = gate_changed = gate_old = gate_new = 0
    for line in output_path.read_text(encoding="utf-8").splitlines():
        item = json.loads(line)
        frame = by_key[(item["run"], item["frame_seq"])]
        original.append(frame)
        new = dict(frame)
        new.update(native_prediction_valid=item["valid"], native_prediction_ns=item["prediction_ns"],
                   native_face_id=item["face"], native_xyza=item["xyza"],
                   native_iteration_converged=item["converged"])
        replayed.append(new)
        invalid += not item["valid"]
        nonconverged += item["valid"] and not item["converged"]
        changed_face += item["valid"] and item["face"] != frame["native_face_id"]
        reference, tolerance = frame.get("gate_reference_yaw_rad"), frame.get("gate_tolerance_rad")
        if item["valid"] and all(analysis.finite(v) for v in [reference, tolerance, frame.get("gate_error_rad")]):
            old = abs(frame["gate_error_rad"]) <= tolerance
            fresh = abs(math.remainder(item["command_yaw"] - reference, math.tau)) <= tolerance
            gate_old += old
            gate_new += fresh
            gate_changed += old != fresh
    # Keep all recorded reference observations and identity transitions. Score
    # only replayed source frames; otherwise subset gaps would bias matching.
    def score(source):
        wanted = {(f["run"], f["frame_seq"]) for f in source}
        replacements = {(f["run"], f["frame_seq"]): f for f in source}
        context = [replacements.get((f["run"], f["frame_seq"]), dict(f, native_prediction_valid=False))
                   for f in frames]
        report = analysis.analyze(context)
        return {(r["run"], r["frame_seq"]): r for r in report["residual_samples"]
                if (r["run"], r["frame_seq"]) in wanted}
    before, after = score(original), score(replayed)
    paired = before.keys() & after.keys()
    result = {"input_log": path.name, "fixtures": len(fixtures), "exclusions": dict(excluded),
              "invalid_new": invalid, "nonconverged_new": nonconverged,
              "changed_future_face": changed_face, "matched_old": len(before), "matched_new": len(after),
              "paired": len(paired),
              "paired_old": analysis.summarize_residuals([before[k] for k in paired]),
              "paired_new": analysis.summarize_residuals([after[k] for k in paired]),
              "paired_by_rotation_sign": {
                  sign: {"old": analysis.summarize_residuals([before[k] for k in paired if before[k]["rotation_sign"] == sign]),
                         "new": analysis.summarize_residuals([after[k] for k in paired if before[k]["rotation_sign"] == sign])}
                  for sign in ["positive", "negative"]},
              "yaw_gate_old_open": gate_old, "yaw_gate_new_open": gate_new, "yaw_gate_changed": gate_changed,
              "limitations": ["Only the yaw qualification gate is compared; cadence, servo feedback and actual firing are not simulated.",
                              "Existing whole-EKF posterior recovered by inverting the next same-epoch CV prior; no refitting.",
                              "Actual new C++ Aimer selection and ballistic iteration, recorded base time and speed.",
                              "Only originally valid/converged medium predictions with recoverable raw state are replayed; gained windows outside that subset are not counted.",
                              "Snapshots assume jumped=true after stable rotation; no tracker association or primary-face policy is replayed.",
                              "PnP consistency is not independent truth or measured hit rate."]}
    print(json.dumps(result, ensure_ascii=False))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--replay-log", type=Path, action="append", default=[])
    parser.add_argument("--replay-report", type=Path)
    args = parser.parse_args()
    root = args.project_root.resolve()
    binary = root / "build/aimer_prediction_trace_test"
    binary.parent.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="aimer-trace-test-") as folder:
        temp = Path(folder)
        (temp / "test.cpp").write_text(HARNESS, encoding="utf-8")
        config = temp / "config.yaml"
        config.write_text("yaw_offset: 0.0\npitch_offset: 6.0\ncomming_angle: 75\nleaving_angle: 15\n"
                          "high_speed_delay_time: 0.085\nlow_speed_delay_time: 0.085\ndecision_speed: 0.8\n",
                          encoding="utf-8")
        sources = ["tasks/auto_aim/aimer.cpp", "tasks/auto_aim/target.cpp", "tasks/auto_aim/armor.cpp",
                   "tools/math_tools.cpp", "tools/trajectory.cpp", "tools/extended_kalman_filter.cpp", "tools/logger.cpp"]
        subprocess.run(["g++", "-std=c++17", "-O1", "-pthread", "-I" + str(root),
                        "-I" + str(root / "io/serial/include"), "-I/usr/include/eigen3", "-I/usr/include/opencv4",
                        str(temp / "test.cpp"), *[str(root / p) for p in sources],
                        "-lopencv_core", "-lyaml-cpp", "-lspdlog", "-lfmt", "-o", str(binary)],
                       check=True, timeout=180)
        subprocess.run([str(binary), str(config)], check=True, timeout=30, cwd=root)
        # Synthetic config has identical Aimer coefficients to the formal
        # config except pitch_offset, which affects pitch only. Use formal
        # config for recorded snapshots and verify the fixed 85 ms delay.
        if args.replay_log:
            results = [replay_log(root, binary, root / "configs/standard.yaml", temp, path.resolve())
                       for path in args.replay_log]
            if args.replay_report:
                report = json.loads(args.replay_report.read_text(encoding="utf-8")) if args.replay_report.exists() else {}
                if not isinstance(report, dict):
                    raise ValueError("replay report must be a JSON object")
                report["whole_ekf_aimer_replay"] = results
                args.replay_report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
