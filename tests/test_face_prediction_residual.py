"""Production C++ residual estimator tests and optional frozen-log replay.

No camera, serial, motor or launcher is opened. Reference labels are used ONLY
after the C++ replay has produced all corrections, to score the frozen trace.
"""
import argparse
import contextlib
from datetime import datetime
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import unittest

from test_lightbar_search import compile_run

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import analyze_prediction_debug as review

DRIVER = r'''
#include "tasks/auto_aim/face_prediction_residual.hpp"
#include "tasks/auto_aim/rotation_model_manager.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
using Monitor = auto_aim::FacePredictionResidual;
using Json = nlohmann::json;
constexpr double rad = 3.14159265358979323846 / 180.;
Monitor::Input input(int tick) {
  Monitor::Input in;
  in.frame_ns = 1000000000LL + tick * 10000000LL;
  in.epoch = 1; in.context_valid = true; in.high_speed_mode = true;
  in.observations_valid = true;
  in.omega_rad_s = -4.5; in.center_range_m = 5.;
  in.forecast_valid = true; in.face = 0;
  in.prediction_ns = in.frame_ns + 500000000;
  in.raw_yaw_rad = -.6 * rad; in.delay_s = .33; in.speed_mps = 20;
  // Another plate remains observed; face 0 has three independent passages.
  in.observations[3] = std::array<double, 2>{5., 0.};
  for (int center : {50, 190, 330, 470, 610})
    if (std::abs(tick - center) <= 2)
      in.observations[0] = std::array<double, 2>{5., 0.};
  return in;
}
Monitor trained() {
  Monitor m({true});
  Monitor::Output out;
  for (int i=0; i<=380; ++i) out=m.update(input(i));
  assert(out.ready && out.cycles == 3 && std::abs(out.correction_rad - .3*rad)<1e-12);
  return m;
}
Monitor::Input phased_input(int tick, int sign = -1, bool constant = false) {
  auto in=input(tick);
  in.omega_rad_s=sign*4.5;
  in.observations[0].reset();
  for(int center : {70,210,350,490,630})
    if(std::abs(tick-center)<=20) in.observations[0]=std::array<double,2>{5.,0.};
  in.forecast_alignment_rad=(-40.+2.*(tick%140))*rad;
  in.raw_yaw_rad=constant ? .8*rad : .2*rad+.008*in.forecast_alignment_rad;
  return in;
}
void tests() {
  { Monitor m({true}); Monitor::Output out;
    for(int i=0;i<=99;++i) out=m.update(input(i));
    assert(!out.ready && out.cycles == 1); // Adjacent frames are one passage.
  }
  { auto m=trained(); auto in=input(381); in.face=1;
    assert(m.update(in).correction_rad == 0); } // Never transfer face bias.
  { auto m=trained(); auto in=input(381); in.omega_rad_s=-3.1;
    assert(m.update(in).ready); // Selected high mode survives the hysteresis band.
    in=input(382); in.high_speed_mode=false;
    assert(m.update(in).correction_rad==0); // Classification is authoritative, even at 4.5rad/s.
  }
  for(int test=0;test<11;++test) {
    auto m=trained(); auto in=input(381);
    if(test==0) in.omega_rad_s=4.5; // Reverse rotation.
    if(test==1) in.epoch=2;
    if(test==2) in.rebuilt=true;
    if(test==3) in.context_valid=false;
    if(test==4) {in.omega_rad_s=-2.; in.high_speed_mode=false;} // Central low-speed bypass.
    if(test==5) in.center_range_m=5.6;
    if(test==6) in.omega_rad_s=-6.1;
    if(test==7) in.delay_s=.335;
    if(test==8) in.speed_mps=22;
    if(test==9) in.prediction_ns+=130000000;
    if(test==10) in.frame_ns-=10000000; // Duplicate timestamp.
    const auto out=m.update(in);
    assert(!out.ready && out.correction_rad==0);
    auto resumed=input(382);
    assert(m.update(resumed).correction_rad==0); // Returning cannot reuse history.
  }
  { auto m=trained(); Monitor::Output out;
    for(int i=381;i<=405;++i) {
      auto in=input(i); in.observations_valid=false; in.observations={};
      out=m.update(in); assert(!out.ready && out.correction_rad==0);
    }
    assert(m.update(input(406)).correction_rad==0); // Loss beyond 200ms clears learning.
  }
  for(int gap_ms : {150,199,200}) {
    auto m=trained();
    const auto before=m.update(input(381));
    const auto last_seen=input(381).frame_ns;
    // Keep processing frames while the armor is temporarily invisible.
    for(int elapsed=10;elapsed<gap_ms;elapsed+=10) {
      auto in=input(381+elapsed/10); in.observations_valid=false; in.observations={};
      const auto out=m.update(in);
      assert(!out.eligible && !out.ready && out.correction_rad==0);
    }
    auto resumed=input(381+gap_ms/10);
    resumed.frame_ns=last_seen+gap_ms*1000000LL;
    resumed.prediction_ns=resumed.frame_ns+500000000;
    const auto out=m.update(resumed);
    if(gap_ms<200) {
      assert(out.ready && out.cycles==before.cycles);
      assert(out.generation==before.generation);
      assert(std::abs(out.correction_rad-before.correction_rad)<1e-12);
    } else {
      assert(!out.ready && out.cycles==0 && out.correction_rad==0);
      assert(out.generation!=before.generation);
    }
  }
  for(int gap_ms : {149,150}) {
    auto m=trained();
    // A processing pause is distinct from a visibility gap: retain its old limit.
    auto in=input(381);
    in.frame_ns=input(380).frame_ns+gap_ms*1000000LL;
    in.prediction_ns=in.frame_ns+500000000;
    const auto out=m.update(in);
    assert(out.ready==(gap_ms<150));
    if(gap_ms>=150) assert(out.correction_rad==0);
  }
  { Monitor a({true}), b({true});
    for(int i=0;i<=700;++i) {
      auto in=input(i), changed=in;
      if(i>=469 && changed.observations[0])
        changed.observations[0]=std::array<double,2>{5., .07};
      const auto x=a.update(in), y=b.update(changed);
      if(i<469) assert(x.correction_rad==y.correction_rad && x.matched==y.matched);
    }
  }
  { Monitor m; for(int i=0;i<400;++i) assert(!m.update(input(i)).eligible); }
  { Monitor m({true});
    for(int i=0;i<=700;++i) {
      auto in=input(i); in.raw_yaw_rad=.8*rad;
      const auto out=m.update(in);
      if(i==380) assert(out.ready && std::abs(out.correction_rad + .3*rad)<1e-12);
    }
  }
  { Monitor m({true}); Monitor::Output out;
    // Three separated fragments inside one turn are still not three turns.
    for(int i=0;i<=150;++i) {
      auto in=input(i); in.observations[0].reset();
      if(i==50 || i==80 || i==110) in.observations[0]=std::array<double,2>{5.,0.};
      out=m.update(in);
    }
    assert(!out.ready && out.cycles<3);
  }
  { Monitor m({true}); Monitor::Output out;
    // Widely inconsistent completed cycles must not become a correction.
    for(int i=0;i<=380;++i) {
      auto in=input(i);
      in.raw_yaw_rad=(i<100 ? -.8 : (i<240 ? .7 : -.1))*rad;
      out=m.update(in);
    }
    assert(out.cycles==3 && !out.ready && out.correction_rad==0);
  }
  { Monitor m({true});
    // No reference within +/-25ms: it must not be extrapolated from face 3.
    for(int i=0;i<400;++i) {
      auto in=input(i); in.observations[0].reset();
      const auto out=m.update(in); assert(!out.ready && out.matched==0);
    }
  }
  for(int sign : {-1, 1}) {
    Monitor m({true}); Monitor::Output out;
    for(int i=0;i<=380;++i) {
      auto in=input(i); in.omega_rad_s=sign*4.5;
      out=m.update(in);
    }
    assert(out.ready && std::abs(out.correction_rad - .3*rad)<1e-12);
  }
  { Monitor m({true}); Monitor::Output out;
    for(int i=0;i<=380;++i) {
      auto in=input(i); in.raw_yaw_rad=179.6*rad;
      if(in.observations[0])
        in.observations[0]=std::array<double,2>{5*std::cos(-179.8*rad),5*std::sin(-179.8*rad)};
      out=m.update(in);
    }
    assert(out.ready && std::abs(out.correction_rad - .3*rad)<1e-12);
  }
  for(bool gap : {false,true}) {
    Monitor m({true}); Monitor::Output out;
    for(int i=0;i<=60;++i) {
      auto in=input(i); in.forecast_valid=(i==0); in.observations[0].reset();
      if(i==(gap ? 47 : 49) || i==(gap ? 53 : 51))
        in.observations[0]=std::array<double,2>{5.,0.};
      out=m.update(in);
      if(i<=52) assert(out.matched==0); // Wait for the full 25ms bracket.
    }
    assert(out.matched==(gap ? 0u : 1u)); // +/-30ms is rejected, +/-10ms accepted.
  }
  { Monitor m({true});
    for(int i=0;i<=1300;++i) {
      auto in=input(i); if(i>335) in.observations[0].reset();
      const auto out=m.update(in);
      if(i==1300) assert(!out.ready); // 8s expiry despite another visible plate.
    }
  }
  { auto m=trained();
    // A huge old frame gap clears pending forecasts as well as learned bias.
    assert(m.update(input(500)).correction_rad==0);
  }
  for(int sign : {-1,1}) {
    Monitor m({true,.3*rad,true});
    for(int i=0;i<400;++i) m.update(phased_input(i,sign));
    auto in=phased_input(400,sign); in.forecast_alignment_rad=-20*rad;
    const auto left=m.update(in);
    in=phased_input(401,sign); in.forecast_alignment_rad=20*rad;
    const auto right=m.update(in);
    assert(left.phase_conditioned && right.phase_conditioned);
    assert(left.phase_cycles==3 && right.phase_cycles==3);
    assert(std::abs(left.correction_rad+.04*rad)<1e-10);
    assert(std::abs(right.correction_rad+.36*rad)<1e-10);
    assert(Monitor::same_model(left,right)); // Moving along a learned curve keeps frame evidence.
    auto replacement=right; ++replacement.model_revision;
    assert(!Monitor::same_model(left,replacement));
    replacement=right; replacement.phase_conditioned=false;
    assert(!Monitor::same_model(left,replacement));
    in=phased_input(402,sign); in.forecast_alignment_rad=60*rad;
    const auto outside=m.update(in);
    assert(!outside.phase_conditioned && std::abs(outside.correction_rad+.2*rad)<1e-10);
    in=phased_input(403,sign); in.forecast_alignment_rad=NAN;
    assert(!m.update(in).phase_conditioned);
    in=phased_input(404,sign); in.high_speed_mode=false;
    assert(m.update(in).correction_rad==0);
  }
  { Monitor a({true,.3*rad,true}), b({true,.3*rad,false});
    for(int i=0;i<=400;++i) {
      auto in=phased_input(i,-1,true);
      if(i==400) in.forecast_alignment_rad=0;
      const auto x=a.update(in), y=b.update(in);
      if(i==400) {
        assert(x.phase_conditioned && std::abs(x.correction_rad+.6*rad)<1e-10);
        assert(!y.phase_conditioned && std::abs(y.correction_rad+.3*rad)<1e-10);
      }
    }
  }
  { Monitor m({true,.3*rad,true}); Monitor::Output out;
    for(int i=0;i<=400;++i) {
      auto in=phased_input(i); in.raw_yaw_rad=.2*rad;
      if(i==400) in.forecast_alignment_rad=0;
      out=m.update(in);
    }
    // Stable local data alone are insufficient: no held-out improvement.
    assert(out.ready && !out.phase_conditioned && std::abs(out.correction_rad+.2*rad)<1e-10);
  }
  for(int reset=0;reset<6;++reset) {
    Monitor m({true,.3*rad,true});
    for(int i=0;i<400;++i) m.update(phased_input(i,-1,true));
    auto in=phased_input(400,-1,true); in.forecast_alignment_rad=0;
    assert(m.update(in).phase_conditioned);
    in=phased_input(401,-1,true); in.forecast_alignment_rad=0;
    if(reset==0) in.epoch=2;
    if(reset==1) in.omega_rad_s=4.5;
    if(reset==2) in.delay_s=.335;
    if(reset==3) in.center_range_m=5.6;
    if(reset==4) in.context_valid=false;
    if(reset==5) in.frame_ns+=200000000;
    const auto out=m.update(in);
    assert(!out.phase_conditioned && out.correction_rad==0);
  }
  { Monitor a({true,.3*rad,true}), b({true,.3*rad,true});
    for(int i=0;i<=700;++i) {
      auto in=phased_input(i), changed=in;
      if(i>=469 && changed.observations[0])
        changed.observations[0]=std::array<double,2>{5., .07};
      const auto x=a.update(in), y=b.update(changed);
      if(i<469) assert(x.correction_rad==y.correction_rad && x.matched==y.matched);
    }
  }
  { Monitor::Output a,b;
    a.generation=100; b.generation=101;
    assert(Monitor::same_model(a,b)); // Inactive/low-speed generations never reset the fire window.
    a.correction_rad=.1*rad;
    assert(!Monitor::same_model(a,b)); // Flat-model changes retain the old rule.
  }
  std::cout << "production residual tests passed (phase, validation, cycles, bounds, causality, lifecycle)\n";
}
int main(int argc, char **argv) {
  if(argc==1) { tests(); return 0; }
  using Manager = auto_aim::RotationModelManager;
  Manager::Config cfg;
  const auto yaml=YAML::LoadFile(argv[2]);
  Monitor::Config monitor_config;
  monitor_config.enabled=true;
  monitor_config.phase_conditioned_enabled=yaml["xuc_high_speed_face_phase_residual_enabled"] &&
    yaml["xuc_high_speed_face_phase_residual_enabled"].as<bool>();
  Monitor monitor(monitor_config);
  monitor_config.phase_conditioned_enabled=false;
  Monitor baseline_monitor(monitor_config);
  cfg.high_speed_phase_enabled=yaml["xuc_high_speed_phase_enabled"].as<bool>();
  cfg.vehicle_enter_rad_s=yaml["xuc_vehicle_model_enter_rad_s"].as<double>();
  cfg.vehicle_exit_rad_s=yaml["xuc_vehicle_model_exit_rad_s"].as<double>();
  cfg.high_speed_enter_rad_s=yaml["xuc_high_speed_enter_rad_s"].as<double>();
  cfg.high_speed_exit_rad_s=yaml["xuc_high_speed_exit_rad_s"].as<double>();
  cfg.mode_dwell_s=yaml["xuc_rotation_mode_dwell_ms"].as<double>()/1000.;
  cfg.tracker_dropout_hold_s=yaml["xuc_rotation_tracker_dropout_hold_ms"].as<double>()/1000.;
  cfg.rate_filter_tau_s=yaml["xuc_rotation_rate_filter_tau_ms"].as<double>()/1000.;
  Manager manager(cfg);
  std::ifstream stream(argv[1]);
  assert(stream.good());
  std::string line;
  while(std::getline(stream,line)) {
    const auto j=Json::parse(line);
    Monitor::Input in;
    in.frame_ns=j.at("frame_ns"); in.epoch=j.at("epoch");
    in.context_valid=j.at("context_valid"); in.rebuilt=j.at("rebuilt");
    in.observations_valid=j.at("observations_valid");
    in.omega_rad_s=j.at("omega_rad_s"); in.center_range_m=j.at("center_range_m");
    if(in.rebuilt) manager.reset();
    const auto mode=manager.update(in.frame_ns/1e9,j.value("model_trusted",true),in.omega_rad_s);
    in.high_speed_mode=j.value("high_speed_mode",mode.motion_mode==Manager::MotionMode::high_speed);
    in.forecast_valid=j.at("forecast_valid"); in.face=j.at("face");
    in.prediction_ns=j.at("prediction_ns"); in.raw_yaw_rad=j.at("raw_yaw_rad");
    in.delay_s=j.at("delay_s"); in.speed_mps=j.at("speed_mps");
    if(j.contains("forecast_alignment_rad") && !j["forecast_alignment_rad"].is_null())
      in.forecast_alignment_rad=j["forecast_alignment_rad"];
    for(int f=0;f<4;++f) if(!j.at("observations")[f].is_null())
      in.observations[f]=j.at("observations")[f].get<std::array<double,2>>();
    const auto out=monitor.update(in);
    const auto baseline=baseline_monitor.update(in);
    std::cout << Json{{"frame_ns",in.frame_ns},{"correction_rad",out.correction_rad},
      {"baseline_correction_rad",baseline.correction_rad},
      {"ready",out.ready},{"eligible",out.eligible},{"cycles",out.cycles},
      {"phase_conditioned",out.phase_conditioned},{"phase_cycles",out.phase_cycles},
      {"model_revision",out.model_revision},
      {"rotation_mode",j.value("rotation_mode",std::string(Manager::name(mode.motion_mode)))},
      {"rotation_mode_trusted",mode.tracker_trusted},
      {"generation",out.generation},{"matched",out.matched},{"unmatched",out.unmatched}}
      .dump() << '\n';
  }
}
'''


class ProductionResidualTests(unittest.TestCase):
    def test_production_estimator(self):
        compile_run(DRIVER, "test_face_prediction_residual", libraries=("yaml-cpp",))

    def test_three_mode_manager(self):
        compile_run('#include "tasks/auto_aim/rotation_model_manager_test.cpp"',
                    "test_rotation_mode_switch")

    def test_phase_curve_keeps_production_frame_evidence(self):
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        start = source.index("struct RotationFireWindow")
        window = source[start:source.index("\n};", start) + 3]
        start = source.index("    const bool face_correction_changed =")
        change = source[start:source.index(";", start) + 1]
        code = r'''
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include "tasks/auto_aim/face_prediction_residual.hpp"
WINDOW
int main() {
  auto_aim::FacePredictionResidual::Output previous_face_residual;
  RotationFireWindow w;
  for(int frame=1;frame<=8;++frame) {
    auto face_residual=previous_face_residual;
    face_residual.correction_rad=frame*.01*M_PI/180.;
    face_residual.phase_conditioned=true; face_residual.ready=true;
    if(frame==3) ++face_residual.model_revision; // New learning requires two fresh frames.
    if(frame>=5) { // Disabled/high->low: the original zero path must accumulate normally.
      face_residual.phase_conditioned=false; face_residual.correction_rad=0;
      ++face_residual.generation;
    }
CHANGE
    previous_face_residual=face_residual;
    w.update(true,face_correction_changed,3,1,4,0,false,frame*.01,0.,
      std::chrono::steady_clock::time_point(std::chrono::milliseconds(20*frame)),
      2,.5,.2*M_PI/180.,frame,frame*.01,4.5,60./57.3,20./57.3);
    assert(w.two_frames_ready==(frame==2 || frame==4 || frame>=6));
  }
}
'''.replace("WINDOW", window).replace("CHANGE", change)
        compile_run(code, "test_face_residual_frame_evidence")


def replay_inputs(frames):
    """Reconstruct inputs from current-frame fields only; never future labels."""
    rows = []
    for f in frames:
        state = f.get("prediction_state_at_frame") or f.get("whole_ekf_state_at_frame")
        point = f.get("native_xyza")
        valid_state = review.vector(state, 11)
        valid_point = review.vector(point, 4)
        alignment = None
        prediction_ns = f.get("native_prediction_ns")
        if valid_state and valid_point and review.finite(prediction_ns) and prediction_ns >= f["frame_ns"]:
            dt = (prediction_ns - f["frame_ns"]) / 1e9
            alignment = math.remainder(point[3] - math.atan2(
                state[2] + state[3] * dt, state[0] + state[1] * dt), math.tau)
        recorded = f.get("face_residual", {}).get("input")
        if recorded:
            recorded = {k: (0 if v is None and k != "forecast_alignment_rad" else v)
                        for k, v in recorded.items()}
            recorded.setdefault("forecast_alignment_rad", alignment)
            rows.append({**recorded, "frame_ns": f["frame_ns"],
                "model_trusted": f.get("rotation_mode_trusted", f.get("tracker_converged", False)),
                **({"rotation_mode": f["rotation_mode"]} if "rotation_mode" in f else {})})
            continue
        observations = [None] * 4
        for o in review.valid_observations(f):
            if 0 <= o["face_id"] < 4:
                observations[o["face_id"]] = o["xyz_world_m"][:2]
        age = ((f.get("aimer_call_end_ns") or f["frame_ns"]) - f["frame_ns"]) / 1e9
        rows.append({
            "frame_ns": f["frame_ns"], "epoch": f.get("track_epoch") or 0,
            "context_valid": bool(f.get("target_present") and valid_state and
                f.get("armor_count") == 4 and f.get("tracker_converged") and
                f.get("frame_imu_valid") and f.get("motion_mode") == "vehicle" and
                f.get("work_zone_target_state") in ("confirming", "tracking", "lost_hold") and
                0 <= age < .15),
            "rebuilt": bool(f.get("track_epoch_reset")),
            "model_trusted": bool(f.get("tracker_converged")),
            "observations_valid": bool(f.get("tracker_converged") and
                f.get("primary_face_id", -1) >= 0 and any(o is not None for o in observations)),
            "observations": observations,
            "omega_rad_s": f.get("omega_rad_s") or 0,
            "center_range_m": math.hypot(state[0], state[2]) if valid_state else 0,
            "forecast_valid": bool(f.get("native_prediction_trusted") and
                f.get("native_prediction_valid") and
                f.get("native_geometry_source") == "whole_ekf_rigid" and valid_point),
            "face": f.get("native_face_id", -1),
            "prediction_ns": f.get("native_prediction_ns") or 0,
            "raw_yaw_rad": math.atan2(point[1], point[0]) if valid_point else 0,
            "delay_s": f.get("native_delay_s") or 0,
            "speed_mps": f.get("speed_mps") or 0,
            "forecast_alignment_rad": alignment,
        })
    return rows


def replay(log, start, end, output, segments=None):
    frames, parse_errors = review.read_frames([log])
    # Preserve all prior frames as warm-up; the requested interval only grades.
    frames = [f for f in frames if f.get("log_time") and f["log_time"][11:] < end]
    if not frames or len({f["run"] for f in frames}) != 1:
        raise ValueError("Select one nonempty recorded run")
    # Run the production C++ monitor FIRST; grading cannot influence its inputs.
    captured = io.StringIO()
    with contextlib.redirect_stdout(captured):
        compile_run(DRIVER, "test_face_prediction_residual", libraries=("yaml-cpp",), fixtures={
            "frames.jsonl": "\n".join(json.dumps(r) for r in replay_inputs(frames)),
            "mode.yaml": (ROOT / "configs/standard.yaml").read_text(encoding="utf-8")},
            arguments=("frames.jsonl", "mode.yaml"))
    results = [json.loads(line) for line in captured.getvalue().splitlines()]
    if len(results) != len(frames):
        raise AssertionError("Replay omitted frame queries")
    corrections = {r["frame_ns"]: math.degrees(r["correction_rad"]) for r in results}
    previous = {r["frame_ns"]: math.degrees(r["baseline_correction_rad"]) for r in results}
    lookup = {(f["run"], f["frame_seq"]): f for f in frames}
    base = review.analyze(frames)
    midpoint = (frames[0]["frame_ns"] + frames[-1]["frame_ns"]) // 2
    samples = []
    for r in base["residual_samples"]:
        f = lookup[(r["run"], r["frame_seq"])]
        if not start <= f["log_time"][11:] < end:
            continue
        recorded = f.get("face_residual", {}).get("correction_rad")
        samples.append({"frame_seq": f["frame_seq"], "face_id": r["face_id"],
            "log_time": f["log_time"],
            "request": bool(f.get("request_shoot")), "second_half": f["frame_ns"] >= midpoint,
            "baseline_deg": r["yaw_error_deg"], "correction_deg": corrections[f["frame_ns"]],
            "existing_deg": r["yaw_error_deg"] + previous[f["frame_ns"]],
            "recorded_deg": r["yaw_error_deg"] + math.degrees(recorded)
                            if review.finite(recorded) else None,
            "corrected_deg": r["yaw_error_deg"] + corrections[f["frame_ns"]]})
    def summary(rows):
        result = {}
        for k in ("baseline_deg", "existing_deg", "recorded_deg", "corrected_deg"):
            values = [r[k] for r in rows if review.finite(r[k])]
            result[k] = review.stats(values)
            if values:
                result[k]["mean_abs"] = sum(abs(v) for v in values) / len(values)
        return result
    segment_reports = {}
    if segments:
        manifest = json.loads(segments.read_text(encoding="utf-8"))
        for segment in manifest["segments"]:
            selected = [r for r in samples if segment["start"] <= r["log_time"] < segment["end"]]
            # Wall-clock boundaries come only from the user's test schedule.
            first, last = datetime.fromisoformat(segment["start"]), datetime.fromisoformat(segment["end"])
            middle = str(first + (last - first) / 2)
            segment_reports[segment["label"]] = {
                "start": segment["start"], "end": segment["end"],
                "all_scored": summary(selected),
                "same_original_requests": summary([r for r in selected if r["request"]]),
                "second_half": summary([r for r in selected if r["log_time"] >= middle])}
    recorded_differences = [abs(r["baseline_correction_rad"] -
        f["face_residual"]["correction_rad"]) for f, r in zip(frames, results)
        if review.finite(f.get("face_residual", {}).get("correction_rad"))]
    report = {"input": str(log.resolve()), "sha256": hashlib.sha256(log.read_bytes()).hexdigest(),
        "window": [start, end], "frames": len(frames), "parse_errors": parse_errors,
        "all_scored": summary(samples),
        "segments": segment_reports,
        "baseline_recording_max_difference_rad": max(recorded_differences, default=None),
        "second_half": summary([r for r in samples if r["second_half"]]),
        "same_original_requests": summary([r for r in samples if r["request"]]),
        "active_frames": sum(r["ready"] for r in results), "queries": results, "samples": samples,
        "phase_conditioned_frames": sum(r["phase_conditioned"] for r in results),
        "recorded_runtime_inputs": sum(bool(f.get("face_residual", {}).get("input")) for f in frames),
        "rotation_modes": {k: sum(r["rotation_mode"] == k for r in results)
                           for k in ("stationary", "low_speed", "high_speed")},
        "rotation_untrusted_frames": sum(not r["rotation_mode_trusted"] for r in results),
        "limitations": [
            "Frozen accepted associations, target states and original request times; no closed-loop or new-request replay.",
            "Future same-face accepted PnP is a shared visual reference, not physical truth or hit rate.",
            "Exploratory replay of existing logs; no independent field validation of the new correction.",
            "Missing runtime inputs are reconstructed from recorded current-frame fields; no future labels enter C++ inputs.",
            "Runtime adds current feedback freshness and solved-pose checks; offline availability may differ."]}
    output.parent.mkdir(exist_ok=True, parents=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k not in ("queries", "samples")},
                     ensure_ascii=False, indent=2))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay-log", type=Path)
    parser.add_argument("--start", default="23:18:15")
    parser.add_argument("--end", default="23:19:00")
    parser.add_argument("--output", type=Path, default=ROOT / "logs/high_speed_runtime_review.json")
    parser.add_argument("--segments", type=Path)
    args, remaining = parser.parse_known_args()
    result = unittest.main(argv=[sys.argv[0], *remaining], exit=False)
    if not result.result.wasSuccessful():
        sys.exit(1)
    if args.replay_log:
        replay(args.replay_log, args.start, args.end, args.output, args.segments)
