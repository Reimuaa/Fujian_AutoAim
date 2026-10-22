"""Causal production qualification/shadow replay. Never opens hardware or changes timing."""
import contextlib
import io
import json
import math
import re
from pathlib import Path
import sys

from test_lightbar_search import compile_run
from test_face_prediction_residual import replay_inputs

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import analyze_prediction_debug as review

DRIVER = r'''
#include "tasks/auto_aim/face_prediction_residual.hpp"
#include "tasks/auto_aim/high_speed_fire_quality.hpp"
#include "tasks/auto_aim/fixed_center_model.hpp"
#include "tasks/auto_aim/robust_range_filter.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tools/trajectory.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using Json=nlohmann::json;
using namespace std::chrono;
using Monitor=auto_aim::FacePredictionResidual;
double number(const Json & j,const char *key,double fallback=NAN) {
  return j.contains(key) && j[key].is_number()?j[key].get<double>():fallback;
}
Json json_number(double x){return std::isfinite(x)?Json(x):Json(nullptr);}
int main(int argc,char **argv) {
  std::ifstream stream(argv[1]);std::string line;
  Monitor native({true,.3*M_PI/180.,true}),shadow({true,.3*M_PI/180.,true});
  auto_aim::HighSpeedPitchGate::Config gc;gc.enabled=true;
  auto_aim::HighSpeedPitchGate pitch(gc);
  auto_aim::FixedCenterModel model;auto_aim::RobustRangeFilter range;
  auto_aim::FixedCenterOwnership owner;
  auto_aim::Aimer aimer(argv[2]);
  while(std::getline(stream,line)) {
    const auto row=Json::parse(line);const auto &j=row["residual"];
    Monitor::Input in;
    in.frame_ns=j["frame_ns"];in.epoch=j["epoch"];in.context_valid=j["context_valid"];
    in.high_speed_mode=row["high"];in.rebuilt=j["rebuilt"];in.observations_valid=j["observations_valid"];
    in.omega_rad_s=number(j,"omega_rad_s");in.center_range_m=number(j,"center_range_m");
    in.forecast_valid=j["forecast_valid"];in.face=j["face"];in.prediction_ns=j["prediction_ns"];
    in.raw_yaw_rad=number(j,"raw_yaw_rad");in.delay_s=number(j,"delay_s");in.speed_mps=number(j,"speed_mps");
    in.forecast_alignment_rad=number(j,"forecast_alignment_rad");
    for(int face=0;face<4;++face)if(j["observations"][face].is_array())
      in.observations[face]=std::array<double,2>{j["observations"][face][0],j["observations"][face][1]};
    const auto quality=native.update(in);
    auto_aim::FixedCenterModel::Input mi;
    mi.frame_ns=in.frame_ns;mi.epoch=in.epoch;mi.active=in.high_speed_mode;
    mi.context_valid=in.context_valid;mi.rebuilt=in.rebuilt;
    if(row["state"].is_array() && row["state"].size()==11) {
      mi.seed.resize(11);for(int i=0;i<11;++i)mi.seed[i]=row["state"][i];
    }
    for(const auto &o:row["observations"]) {
      auto_aim::FixedCenterModel::Observation mo;mo.face=o["face"];
      for(int k=0;k<3;++k)mo.xyz[k]=o["xyz"][k];mi.observations.push_back(mo);
    }
    const auto begin=steady_clock::now();
    const auto fitted=model.update(mi);
    double shadow_yaw=NAN;int shadow_face=-1;int64_t shadow_ns=0;
    bool shadow_converged=false, candidate_ready=false;
    Eigen::Vector4d point=Eigen::Vector4d::Constant(NAN);
    if(fitted.ready && !mi.observations.empty() && std::isfinite(number(row,"base_horizon_s")) &&
        std::isfinite(number(row,"delay_s"))) {
      auto_aim::Armor a(9,.8f,cv::Rect(0,0,100,40),{{0,0},{100,0},{100,40},{0,40}});
      a.name=auto_aim::ArmorName::three;a.type=auto_aim::ArmorType::small;
      a.priority=auto_aim::ArmorPriority::first;a.xyz_in_world=mi.observations.front().xyz;
      a.ypr_in_world={mi.seed[6],0,0};
      Eigen::VectorXd p(11);p.setOnes();auto_aim::YawESKF::Config yaw;yaw.enabled=false;
      // Reuse the recorded base prediction horizon, including its unchanged
      // configured delay. No search or adaptation of delay/speed is performed.
      const double process_s=number(row,"base_horizon_s")-number(row,"delay_s");
      const auto timestamp=steady_clock::now()-nanoseconds(static_cast<int64_t>(process_s*1e9));
      auto_aim::Target base(a,timestamp,.3,4,p,yaw);
      auto candidate=base.with_fixed_center_prediction(fitted.state);
      if(candidate) {
        candidate_ready=true;
        auto_aim::AimPredictionTrace trace;
        const auto cmd=aimer.aim({*candidate},timestamp,in.speed_mps,true,&trace,
          auto_aim::AimGeometry::whole_ekf,auto_aim::AimTiming::high_speed);
        if(cmd.control && trace.valid && trace.converged && aimer.debug_aim_point.valid) {
          shadow_converged=true;
          shadow_yaw=cmd.yaw;shadow_face=trace.face_id;point=aimer.debug_aim_point.xyza;
          shadow_ns=in.frame_ns+trace.prediction_ns-duration_cast<nanoseconds>(
            timestamp.time_since_epoch()).count();
        }
      }
    }
    const double model_us=duration<double,std::micro>(steady_clock::now()-begin).count();
    auto si=in; // No fresh forecast during a gap, but accepted references/history remain causal.
    si.forecast_valid=std::isfinite(shadow_yaw);
    si.raw_yaw_rad=si.forecast_valid?std::atan2(point[1],point[0]):NAN;
    si.face=shadow_face;si.prediction_ns=shadow_ns;
    si.omega_rad_s=fitted.ready?fitted.state[7]:in.omega_rad_s;
    si.center_range_m=fitted.ready?std::hypot(fitted.state[0],fitted.state[2]):in.center_range_m;
    si.forecast_alignment_rad=fitted.ready && si.forecast_valid?std::remainder(point[3]-
      std::atan2(fitted.state[2],fitted.state[0]),2*M_PI):NAN;
    const auto corrected_shadow=shadow.update(si);
    const auto ownership=owner.update({in.frame_ns,in.epoch,fitted.generation,
      in.high_speed_mode,in.context_valid,in.rebuilt,candidate_ready});
    if(ownership.changed)pitch=auto_aim::HighSpeedPitchGate(gc);
    const auto selected_quality=ownership.fixed?corrected_shadow:quality;
    const auto rr=range.update(in.frame_ns/1e9,number(row,"model_range"),number(row,"raw_range"));
    double residual=NAN;
    if(row["pitch_observation"].is_array()) {
      const auto &o=row["pitch_observation"];const double x=o[0],y=o[1],z=o[2];
      const double raw=std::hypot(x,y),d=rr.valid?rr.range_m:raw;
      const double scale=d/raw, muzzle=std::hypot(x*scale+.06,y*scale);
      tools::Trajectory reference(10.,.97,.04),current(20.,muzzle,-z+.04);
      if(!reference.unsolvable && !current.unsolvable) {
        const double b=std::clamp(std::atan2(.08639,.97)-reference.pitch+current.pitch-
          std::atan2(-z,d)+.75*M_PI/180.,0.,20*M_PI/180.);
        residual=number(row,"raw_pixel_pitch")-b;
      }
    }
    const auto pq=pitch.update({in.frame_ns,in.epoch,in.high_speed_mode,
      row["pitch_context"],residual,number(row,"pitch_tracking")});
    const bool pred_open=auto_aim::past_forecast_quality_allows(in.high_speed_mode,
      selected_quality.quality_ready,selected_quality.eligible,
      selected_quality.past_error_bound_rad,.45*M_PI/180.);
    Json out={{"pitch_open",pq.open},{"pitch_residual_rad",json_number(residual)},
      {"pitch_variation_rad",json_number(pq.variation_rad)},{"prediction_open",pred_open},
      {"quality_ready",quality.quality_ready},{"quality_cycles",quality.quality_cycles},
      {"quality_bound_rad",json_number(quality.past_error_bound_rad)},
      {"correction_rad",quality.correction_rad},{"shadow_ready",fitted.ready},
       {"source_fixed",ownership.fixed},{"source_available",ownership.available},
       {"source_changed",ownership.changed},{"model_generation",fitted.generation},
       {"shadow_converged",shadow_converged},
       {"selected_correction_rad",selected_quality.correction_rad},
      {"shadow_model_us",model_us},{"shadow_yaw_rad",json_number(shadow_yaw)},
      {"shadow_correction_rad",corrected_shadow.correction_rad},
      {"shadow_face",shadow_face},{"shadow_prediction_ns",shadow_ns}};
    if(fitted.ready)out["shadow_state"]=std::vector<double>(fitted.state.data(),fitted.state.data()+11);
    std::cout<<out.dump()<<'\n';
  }
}
'''


def replay(paths, output):
    frames, parse_errors = review.read_frames(paths)
    inputs = replay_inputs(frames)
    rows = []
    run_keys = {}
    for f, r in zip(frames, inputs):
        run_keys.setdefault(f["run"], len(run_keys) + 1)
        r["epoch"] = (run_keys[f["run"]] << 32) + r["epoch"]
        obs = review.valid_observations(f)
        primary = next((o for o in obs if o["face_id"] == f.get("primary_face_id")), None)
        state = f.get("whole_ekf_state_at_frame")
        post = f.get("state_at_frame")
        modeled = review.rigid_face(post, f.get("primary_face_id"), 4)
        pixel = None
        if primary and primary.get("corners_px"):
            pixel = math.atan2(sum(p[1] for p in primary["corners_px"]) / 4 - 540,
                               2365.2176578132576)
        tracking = None
        if review.finite(f.get("output_pitch_rad")) and review.finite(f.get("feedback_world_pitch_rad")):
            tracking = f["output_pitch_rad"] - f["feedback_world_pitch_rad"]
        rows.append({"residual": r, "high": f.get("rotation_mode") == "high_speed",
                     "state": state, "observations": [{"face": o["face_id"], "xyz": o["xyz_world_m"]} for o in obs],
                     "raw_range": math.hypot(*primary["xyz_world_m"][:2]) if primary else None,
                     "model_range": math.hypot(*modeled[:2]) if modeled else None,
                     "pitch_observation": primary.get("xyz_gimbal_m") if primary else None,
                     "raw_pixel_pitch": pixel, "pitch_tracking": tracking,
                     "pitch_context": bool(primary and f.get("frame_imu_valid") and f.get("output_control") and
                                           not f.get("work_zone_inhibit_fire")),
                      "base_horizon_s": ((f["native_base_prediction_ns"] - f["frame_ns"]) / 1e9
                                         if f.get("native_base_prediction_ns") else
                                         ((f["aimer_call_begin_ns"]-f["frame_ns"])/1e9 + f["native_delay_s"]
                                          if f.get("aimer_call_begin_ns") and review.finite(f.get("native_delay_s"))
                                          else None)),
                     "delay_s": f.get("native_delay_s")})
    config = (ROOT / "configs/standard.yaml").read_text(encoding="utf-8")
    # Recorded timing is immutable replay input, including when the user has
    # since changed the formal setting. No delay/speed search is performed.
    delays = {f["native_delay_s"] for f in frames if f.get("rotation_mode") == "high_speed"
              and review.finite(f.get("native_delay_s"))}
    if len(delays) != 1:
        raise ValueError("Replay logs with one unchanged recorded high-speed delay")
    config, count = re.subn(r"(?m)^high_speed_delay_time:\s*[-+0-9.eE]+",
                           "high_speed_delay_time: " + str(delays.pop()), config)
    assert count == 1
    captured = io.StringIO()
    with contextlib.redirect_stdout(captured):
        compile_run(DRIVER, "replay_high_speed_qualification", sources=(
            "tasks/auto_aim/aimer.cpp", "tasks/auto_aim/target.cpp", "tasks/auto_aim/armor.cpp",
            "tools/math_tools.cpp", "tools/trajectory.cpp", "tools/extended_kalman_filter.cpp",
            "tools/logger.cpp"), libraries=("opencv_core", "opencv_imgproc", "yaml-cpp", "spdlog", "fmt"),
            fixtures={"frames.jsonl": "\n".join(json.dumps(row) for row in rows), "config.yaml": config},
            arguments=("frames.jsonl", "config.yaml"))
    results = [json.loads(s) for s in captured.getvalue().splitlines() if s.startswith('{')]
    assert len(results) == len(frames)
    samples = []
    summaries = {}
    references = {}
    from collections import defaultdict
    points = defaultdict(list)
    for f in frames:
        for o in review.valid_observations(f):
            points[(f["run"], f["track_epoch"], o["face_id"])].append((f["frame_ns"], o["xyz_world_m"]))
    references = {k: review.series(v) for k, v in points.items()}
    original = {}
    for path in paths:
        p = path.with_name(path.stem + "_single_face_review.json")
        if p.exists():
            for s in json.loads(p.read_text(encoding="utf-8"))["samples"]:
                original[(s["run"], s["frame_seq"])] = s
        elif path.name == "2026-10-22_12-27-40.log":
            older = json.loads((ROOT / "logs/latest_translation_prediction_review.json").read_text(encoding="utf-8"))
            run = next(f["run"] for f in frames if f.get("log_time", "").startswith("2026-10-22 12:27"))
            for s in older["samples"]:
                original[(run, s["frame_seq"])] = {"yaw_error_deg": s["raw_yaw_error_deg"]}
    for f, r in zip(frames, results):
        if f.get("rotation_mode") != "high_speed":
            assert r["pitch_open"] and r["prediction_open"] and not r["shadow_ready"]
        sample = {"run": f["run"], "frame_seq": f["frame_seq"], "high": f.get("rotation_mode") == "high_speed",
                  "request": bool(f.get("request_shoot")), "frame_ns": f["frame_ns"],
                  "auto": bool(f.get("fire_gates", {}).get("auto_mode")), **r}
        if r["shadow_ready"]:
            assert all(r["shadow_state"][i] == 0 for i in (1, 3, 5))
        reference = references.get((f["run"], f["track_epoch"], r["shadow_face"]))
        if reference and review.finite(r["shadow_yaw_rad"]):
            xyz, _, reason = review.interpolate(reference, r["shadow_prediction_ns"], 25_000_000)
            if xyz is not None:
                error = math.degrees(math.remainder(r["shadow_yaw_rad"] - math.atan2(xyz[1], xyz[0]), math.tau))
                sample["shadow_raw_error_deg"] = error
                sample["shadow_corrected_error_deg"] = error + math.degrees(r["shadow_correction_rad"])
        native_reference = references.get((f["run"], f["track_epoch"], f.get("native_face_id", -1)))
        if native_reference and review.finite(f.get("native_prediction_ns")):
            xyz, _, _ = review.interpolate(native_reference, f["native_prediction_ns"], 25_000_000)
            if xyz is not None and review.finite(f.get("native_yaw_rad")):
                error = math.degrees(math.remainder(f["native_yaw_rad"] - math.atan2(xyz[1], xyz[0]), math.tau))
                sample["native_raw_error_deg"] = error
                sample["native_corrected_error_deg"] = error + math.degrees(
                    f.get("face_residual", {}).get("correction_rad", r["correction_rad"]))
                if r["shadow_ready"]:
                    state = list(r["shadow_state"])
                    state[6] += state[7] * (f["native_prediction_ns"]-f["frame_ns"]) / 1e9
                    point = review.rigid_face(state, f["native_face_id"], 4)
                    sample["shadow_same_face_time_raw_error_deg"] = math.degrees(math.remainder(
                        math.atan2(point[1], point[0])-math.atan2(xyz[1], xyz[0]), math.tau))
        prior = original.get((f["run"], f["frame_seq"]))
        if prior:
            sample["native_raw_error_deg"] = prior["yaw_error_deg"]
            sample["native_corrected_error_deg"] = prior["yaw_error_deg"] + math.degrees(r["correction_rad"])
        samples.append(sample)
    for run in run_keys:
        rr = [s for s in samples if s["run"] == run and s["high"]]
        requests = [s for s in rr if s["request"]]
        paired = [s for s in rr if "shadow_raw_error_deg" in s and "native_raw_error_deg" in s]
        summaries[str(run)] = {"high_frames": len(rr), "original_requests": len(requests),
            "pitch_retained_requests": sum(s["pitch_open"] for s in requests),
            "prediction_retained_requests": sum(s["prediction_open"] for s in requests),
            "both_retained_requests": sum(s["pitch_open"] and s["prediction_open"] for s in requests),
            "quality_ready_frames": sum(s["quality_ready"] for s in rr),
            "shadow_ready_frames": sum(s["shadow_ready"] for s in rr),
            "fixed_owned_frames": sum(s["source_fixed"] for s in rr),
            "source_switches": sum(s["source_changed"] for s in rr),
            "fixed_owned_unavailable_frames": sum(s["source_fixed"] and not s["source_available"] for s in rr),
            "converged_shadow_requests": sum(s["shadow_converged"] for s in requests),
            "shadow_model_us": review.stats([s["shadow_model_us"] for s in rr]),
            "paired_predictions": len(paired),
            **{k: review.stats([s.get(k) for s in paired]) for k in ["native_raw_error_deg", "native_corrected_error_deg",
                                                                  "shadow_raw_error_deg", "shadow_corrected_error_deg"]}}
    report = {"summaries": summaries, "frames": len(frames), "parse_errors": parse_errors,
              "non_high_frames_verified_bypass": sum(not s["high"] for s in samples), "samples": samples,
              "limitations": ["Frozen accepted associations; candidate completes its own Aimer face/flight-time iteration and past residual learning.",
                               "Frozen original requests are diagnostic query times; no new request sequence, gimbal closed loop or physical hit rate is simulated.",
                              "Pitch residual reconstructed with production range/ballistic formulas; initial non-high CV range is approximated.",
                               "Same-face future PnP references share calibration; only recorded timing is replayed, not the current formal delay."]}
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "samples"}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    replay([Path(p) for p in sys.argv[2:]], Path(sys.argv[1]))
