#!/usr/bin/env python3
"""Pure offline checks for same-face visual matching and timestamped raw-angle feedback."""
import copy
import importlib.util
import math
import json
import shutil
import subprocess
import tempfile
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location(
    "prediction_analysis", Path(__file__).resolve().parents[1] / "scripts/analyze_prediction_debug.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def fixture():
    result = []
    for i in range(5):
        t = 1_000_000_000 + i * 20_000_000
        row = {"run": 1, "frame_seq": i, "frame_ns": t, "track_epoch": 1,
               "target_present": True, "tracker_converged": True, "frame_imu_valid": True,
               "native_prediction_valid": False,
               "observations": [{"face_id": 0, "xyz_world_m": [4., i * .02, .1],
                                 "body_yaw_rad": math.radians(179 if i == 1 else -179)}],
               "feedback_pose_ns": t, "feedback_world_yaw_rad": -.2,
               "feedback_world_pitch_rad": .1, "feedback_imu_yaw_scale": -2.,
               "feedback_imu_pitch_scale": 1., "feedback_imu_yaw_offset_rad": 0.,
               "feedback_imu_pitch_offset_rad": 0.}
        result.append(row)
    result[0].update(native_prediction_valid=True, native_prediction_trusted=True,
                     native_iteration_converged=True, native_prediction_ns=1_030_000_000,
                     native_face_id=0, native_xyza=[4., .03, .1, math.pi], omega_rad_s=-1.5,
                     tx_ok=True, tx_control=True, tx_write_complete_ns=1_010_000_000,
                     tx_raw_yaw_rad=.1, tx_raw_pitch_rad=.1)
    return result


class PredictionReviewTests(unittest.TestCase):
    def test_same_face_interpolation_and_raw_coordinates(self):
        report = MODULE.analyze(fixture())
        self.assertEqual(report["matched_predictions"], 1)
        self.assertAlmostEqual(report["prediction_error"]["xyz_error_cm"]["max"], 0.)
        self.assertAlmostEqual(report["prediction_error"]["phase_error_deg"]["max"], 0.)
        self.assertAlmostEqual(report["simultaneous_tracking_error"]["yaw_error_deg"]["max"], 0.)

    def test_other_face_is_never_substituted(self):
        frames = fixture()
        frames[0]["native_face_id"] = 1
        report = MODULE.analyze(frames)
        self.assertEqual(report["matched_predictions"], 0)
        self.assertEqual(report["prediction_exclusions"]["same_face_no_bracket"], 1)

    def test_reset_and_incomplete_run_are_counted(self):
        frames = fixture()
        frames[2]["track_epoch"] = 2
        report = MODULE.analyze(frames)
        self.assertEqual(report["prediction_exclusions"]["track_epoch_changed"], 1)
        frames = fixture()
        frames[0]["native_prediction_ns"] = 2_000_000_000
        self.assertEqual(MODULE.analyze(frames)["prediction_exclusions"]["run_ended_before_prediction"], 1)

    def test_no_extrapolation_or_cross_gap_match(self):
        data = ([10, 20], [[1.], [2.]])
        self.assertEqual(MODULE.interpolate(data, 9, 10)[2], "no_bracket")
        frames = fixture()
        frames[1]["observations"] = []
        frames[2]["observations"] = []
        self.assertEqual(MODULE.analyze(frames)["prediction_exclusions"]["same_face_observation_gap"], 1)

    def test_invalid_imu_and_duplicate_face_are_not_references(self):
        for mode in ["invalid_imu", "duplicate"]:
            frames = fixture()
            for f in frames:
                if mode == "invalid_imu":
                    f["frame_imu_valid"] = False
                else:
                    f["observations"].append(copy.deepcopy(f["observations"][0]))
            self.assertEqual(MODULE.analyze(frames)["matched_predictions"], 0)

    def test_runs_do_not_mix(self):
        frames = fixture()
        for f in frames[1:]:
            f["run"] = 2
        self.assertEqual(MODULE.analyze(frames)["matched_predictions"], 0)

    def test_rigid_geometry_alternating_radius_and_height(self):
        state = [4., 0., .2, 0., .1, 0., 0., 1.5, .3, .05, .02]
        self.assertEqual(MODULE.rigid_face(state, 0, 4), [3.7, .2, .1, 0.])
        odd = MODULE.rigid_face(state, 1, 4)
        self.assertAlmostEqual(odd[0], 4.)
        self.assertAlmostEqual(odd[1], -.15)
        self.assertAlmostEqual(odd[2], .12)
        self.assertAlmostEqual(MODULE.rigid_face(state, 1, 2)[2], .1)
        self.assertIsNone(MODULE.rigid_face(state, 4, 4))

    def test_prior_posterior_and_dual_face_consistency(self):
        f = fixture()[0]
        state = [4., 0., 0., 0., .1, 0., math.radians(179), 1.5, .3, 0., 0.]
        f.update(armor_count=4, state_at_frame=state, state_before_update=state.copy())
        f['state_before_update'][0] += .1
        f['observations'] = []
        for face in [0, 1]:
            p = MODULE.rigid_face(state, face, 4)
            f['observations'].append({'face_id': face, 'xyz_world_m': p[:3], 'body_yaw_rad': p[3]})
        report = MODULE.analyze_attribution([f], [])
        self.assertAlmostEqual(report['current_posterior_vs_pnp']['xyz_error_cm']['max'], 0.)
        self.assertAlmostEqual(report['current_prior_vs_pnp']['xyz_error_cm']['mean'], 10.)
        self.assertAlmostEqual(report['dual_face_same_frame']['equivalent_body_phase_difference_deg']['mean'], 0.)
        f['observations'][1]['body_yaw_rad'] += math.radians(12)
        report = MODULE.analyze_attribution([f], [])
        self.assertAlmostEqual(report['dual_face_same_frame']['equivalent_body_phase_difference_deg']['mean'], -12.)

    def test_command_chain_uses_explicit_protocol_scale(self):
        f = fixture()[0]
        f.update(tx_yaw_scale=-2., tx_pitch_scale=1., output_yaw_rad=.1, output_pitch_rad=.2,
                 tx_raw_yaw_rad=-.24, tx_raw_pitch_rad=.2, tx_shoot=1,
                 yaw_before_final_rate_rad=.2, yaw_after_final_rate_rad=.1)
        report = MODULE.analyze_attribution([f], [])['command_chain']['shoot_high']
        self.assertAlmostEqual(report['packed_minus_output_yaw_deg']['mean'], math.degrees(.02))
        self.assertAlmostEqual(report['final_rate_change_deg']['mean'], math.degrees(-.1))
        self.assertAlmostEqual(report['packed_minus_output_pitch_deg']['mean'], 0.)
        del f['tx_yaw_scale']
        self.assertNotIn('packed_minus_output_yaw_deg', MODULE.analyze_attribution([f], [])['command_chain']['shoot_high'])

    def test_incremental_error_does_not_substitute_another_face(self):
        fs = fixture()
        for f in fs:
            f.update(armor_count=4, state_at_frame=[4., 0., 0., 0., .1, 0., 0., 0., .3, 0., 0.])
        report = MODULE.analyze(fs)
        self.assertEqual(report['attribution']['future_minus_current_error']['yaw_future_minus_current_deg']['n'], 1)
        fs[0]['observations'] = []
        report = MODULE.analyze(fs)
        self.assertEqual(report['matched_predictions'], 1)
        self.assertEqual(report['attribution']['future_minus_current_error'], {})

    def test_calibration_conflicts_and_legacy_log_are_explicit(self):
        contexts, conflicts = MODULE.calibration_context([
            {'run': 1, 'calibration': {'x': 1}}, {'run': 2, 'calibration': {'x': 2}},
            {'run': 1, 'calibration': {'x': 3}}])
        self.assertEqual(conflicts, {1})
        self.assertEqual(contexts[2], {'x': 2})
        report = MODULE.analyze(fixture())
        self.assertEqual(report['attribution']['current_prior_vs_pnp'], {})
        self.assertGreater(report['attribution']['exclusions']['state_before_update_missing_or_invalid'], 0)

    def test_duplicate_face_is_excluded_from_current_model(self):
        fs = fixture()
        for f in fs:
            f['observations'].append(copy.deepcopy(f['observations'][0]))
            f.update(armor_count=4, state_at_frame=[4., 0., 0., 0., .1, 0., 0., 0., .3, 0., 0.])
        self.assertEqual(MODULE.analyze(fs)['attribution']['current_posterior_vs_pnp'], {})

    def test_legacy_prior_joins_only_exact_frame_and_never_replaces_schema2(self):
        f = fixture()[0]
        f['kind'] = 'prediction_frame'
        keys = ['center_x_m', 'center_vx_mps', 'center_y_m', 'center_vy_mps', 'center_z_m', 'center_vz_mps',
                'body_yaw_deg', 'raw_omega_rad_s', 'r1_m', 'r2_delta_m', 'height_delta_m']
        d = {'run': 1, 'frame_seq': 0, 'tracker_update_attempted': True,
             'tracker_state_prior': dict(zip(keys, [4., 0., 0., 0., .1, 0., 90., 1.5, .3, 0., 0.]))}
        with tempfile.TemporaryDirectory(prefix='prediction-log-test-') as folder:
            path = Path(folder) / 'test.log'
            def load():
                path.write_text('[2026-10-10 16:50:00.000] [SHOT_DEBUG] ' + json.dumps(f) + '\n' +
                                '[2026-10-10 16:50:00.000] [INFANTRY_DIAG] ' + json.dumps(d) + '\n', encoding='utf-8')
                return MODULE.read_frames([path])[0][0]
            self.assertAlmostEqual(load()['state_before_update'][6], math.pi / 2)
            d['frame_seq'] = 1
            self.assertNotIn('state_before_update', load())
            d['frame_seq'] = 0
            f['state_before_update'] = None
            self.assertIsNone(load()['state_before_update'])


class DualGeometryTests(unittest.TestCase):
    def synthetic_rotation(self, direction=1):
        frames = []
        for i in range(240):
            phase = math.radians(175) + direction * i * .032
            state = [4., 0., .2, 0., .1, 0., phase, direction * 1.6, .3, .05, .02]
            f = fixture()[0]
            f.update(frame_seq=i, frame_ns=1_000_000_000 + i * 20_000_000,
                     armor_count=4, state_at_frame=state.copy(), state_before_update=state.copy(),
                     omega_rad_s=direction * 1.6, observations=[])
            f['state_before_update'][0] += .1
            f['state_before_update'][6] += math.radians(5)
            for face in [i // 50 % 4, (i // 50 + 1) % 4]:
                p = MODULE.rigid_face(state, face, 4)
                f['observations'].append({'face_id': face, 'xyz_world_m': p[:3],
                    'body_yaw_rad': p[3], 'pnp_yaw_before_optimization_rad': p[3]})
            frames.append(f)
        return frames

    def test_unequal_radii_chord_phase_and_wrap_in_both_directions(self):
        for direction in [-1, 1]:
            for f in self.synthetic_rotation(direction)[::17]:
                left, right = MODULE.valid_observations(f)
                row, center = MODULE.dual_center_geometry(left, right, f['state_before_update'])
                self.assertAlmostEqual(row['orientation_center_gap_cm'], 0., places=10)
                self.assertAlmostEqual(row['position_chord_length_error_cm'], 0., places=10)
                self.assertAlmostEqual(row['position_phase_minus_left_yaw_deg'], 0., places=10)
                self.assertAlmostEqual(row['position_phase_minus_right_yaw_deg'], 0., places=10)
                self.assertAlmostEqual(center[0], 4.)
                self.assertAlmostEqual(center[1], .2)
                self.assertAlmostEqual(row['position_center_minus_prior_cm'], 10.)
                for side, obs in [('left', left), ('right', right)]:
                    self.assertAlmostEqual(row[side + '_implied_radius_m'], .35 if obs['face_id'] % 2 else .3)

    def test_yaw_bias_is_separate_from_position_chord(self):
        f = self.synthetic_rotation()[0]
        left, right = MODULE.valid_observations(f)
        right['body_yaw_rad'] += math.radians(12)
        row, center = MODULE.dual_center_geometry(left, right, f['state_before_update'])
        self.assertGreater(row['orientation_center_gap_cm'], 5.)
        self.assertAlmostEqual(row['position_phase_minus_right_yaw_deg'], -12.)
        self.assertAlmostEqual(row['position_chord_length_error_cm'], 0.)
        self.assertAlmostEqual(center[0], 4.)
        raw, _ = MODULE.dual_center_geometry(left, right, f['state_before_update'], yaw_key='pnp_yaw_before_optimization_rad')
        self.assertAlmostEqual(raw['orientation_center_gap_cm'], 0., places=10)

    def test_degenerate_and_nonphysical_geometry_are_explicit(self):
        f = self.synthetic_rotation()[0]
        left, right = MODULE.valid_observations(f)
        right['body_yaw_rad'] = left['body_yaw_rad']
        row, _ = MODULE.dual_center_geometry(left, right, f['state_before_update'])
        self.assertNotIn('left_implied_radius_m', row)
        f['state_before_update'][8] = -.1
        with self.assertRaisesRegex(ValueError, 'nonpositive_recorded_radius'):
            MODULE.dual_center_geometry(left, right, f['state_before_update'])
        f = self.synthetic_rotation()[0]
        left, right = MODULE.valid_observations(f)
        right['xyz_world_m'] = left['xyz_world_m'].copy()
        with self.assertRaisesRegex(ValueError, 'degenerate_position_chord'):
            MODULE.dual_center_geometry(left, right, f['state_before_update'])
        f = self.synthetic_rotation()[0]
        left, right = MODULE.valid_observations(f)
        left['body_yaw_rad'] = math.remainder(left['body_yaw_rad'] + math.pi, math.tau)
        row, _ = MODULE.dual_center_geometry(left, right, f['state_before_update'])
        self.assertEqual(row['implied_radius_nonpositive'], 1)
        self.assertLess(row['left_implied_radius_m'], 0.)

    def test_constant_center_holdout_recovers_geometry_not_biased_prior(self):
        for direction in [-1, 1]:
            frames = self.synthetic_rotation(direction)
            report = MODULE.constant_center_holdout(frames)
            self.assertEqual(report['status'], 'ok')
            self.assertEqual(report['train_frames'] + report['test_frames'], len(frames))
            for observed, expected in zip(report['center_world_xy_m'] + report['radii_even_odd_m'], [4., .2, .3, .35]):
                self.assertAlmostEqual(observed, expected, places=9)
            self.assertLess(report['heldout_residuals']['xy_error_cm']['abs_p95'], 1e-8)
            self.assertGreater(report['paired_heldout_prior']['xy_error_cm']['abs_p95'], 5.)

    def test_holdout_does_not_train_on_test_observations(self):
        frames = self.synthetic_rotation()
        expected = MODULE.constant_center_holdout(frames)
        for f in frames:
            if ((f['frame_ns'] - frames[0]['frame_ns']) // 1_000_000_000) % 2:
                for o in f['observations']:
                    o['xyz_world_m'][1] += .2
        report = MODULE.constant_center_holdout(frames)
        self.assertEqual(report['center_world_xy_m'], expected['center_world_xy_m'])
        self.assertEqual(report['radii_even_odd_m'], expected['radii_even_odd_m'])
        self.assertAlmostEqual(report['heldout_residuals']['xy_error_cm']['abs_p95'], 20.)

    def test_epochs_truncation_and_raw_missing_never_mix_or_substitute(self):
        frames = self.synthetic_rotation()
        for f in frames[120:]:
            f['track_epoch'] = 2
            for o in f['observations']:
                o['xyz_world_m'][0] += 1.
            f['state_before_update'][0] += 1.
            f['state_at_frame'][0] += 1.
        frames[0]['observations_truncated'] = True
        for f in frames[:120]:
            for o in f['observations']:
                del o['pnp_yaw_before_optimization_rad']
        report = MODULE.analyze_dual_geometry(frames)
        self.assertEqual(report['pair_count'], 239)
        self.assertEqual(report['exclusions']['truncated_observations'], 1)
        self.assertEqual(report['raw_yaw_geometry']['orientation_center_gap_cm']['n'], 120)
        fits = report['constant_center_holdout']
        self.assertEqual(len(fits), 2)
        self.assertEqual(fits['run1/epoch1']['raw_yaw']['status'], 'excluded')
        self.assertAlmostEqual(fits['run1/epoch1']['optimized_yaw']['center_world_xy_m'][0], 4.)
        self.assertAlmostEqual(fits['run1/epoch2']['optimized_yaw']['center_world_xy_m'][0], 5.)
        other_run = copy.deepcopy(frames[120:])
        for f in other_run:
            f['run'] = 2
        self.assertEqual(len(MODULE.analyze_dual_geometry(frames + other_run)['constant_center_holdout']), 3)

    def test_rank_deficiency_short_runs_and_constant_correlations(self):
        frames = self.synthetic_rotation()
        for f in frames:
            for o in f['observations']:
                o['body_yaw_rad'] = 0.
        self.assertEqual(MODULE.constant_center_holdout(frames)['reason'], 'rank_deficient_constant_center_fit')
        self.assertEqual(MODULE.constant_center_holdout(frames[:10])['reason'], 'insufficient_train_or_test_frames')
        self.assertIsNone(MODULE.correlation([{'x': 1., 'y': i} for i in range(4)], 'x', 'y')['pearson'])


class PnpReplayTests(unittest.TestCase):
    def setUp(self):
        try:
            import cv2
            import numpy
        except ImportError:
            self.skipTest('Offline OpenCV/NumPy dependencies unavailable on this host')
        self.cv, self.np = cv2, numpy

    def synthetic(self):
        cv, np = self.cv, self.np
        calibration = {'solver_model': 'ippe_fixed_pitch_yaw_grid_v1',
                       'camera_matrix': [2366., 0., 727., 0., 2365., 550., 0., 0., 1.],
                       'distort_coeffs': [.01, -.03, .001, .002, .01],
                       'R_camera2gimbal': [0., 0., 1., -1., 0., 0., 0., -1., 0.],
                       't_camera2gimbal': [.1, .01, -.03]}
        rot = cv.Rodrigues(np.array([.01, -.02, .08]))[0]
        rcg = np.asarray(calibration['R_camera2gimbal']).reshape(3, 3)
        yaw, pitch = math.atan2(rot[1, 0], rot[0, 0]) + math.radians(20), math.radians(15)
        cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
        rw = np.array([[cy*cp, -sy, cy*sp], [sy*cp, cy, sy*sp], [-sp, 0, cp]])
        obj = np.array([[0, .0675, .028], [0, -.0675, .028], [0, -.0675, -.028], [0, .0675, -.028]], dtype=np.float32)
        xyz = np.array([3.8, .1, .03])
        cr = rcg.T @ rot.T @ rw
        ct = rcg.T @ (rot.T @ xyz - np.asarray(calibration['t_camera2gimbal']))
        k = np.asarray(calibration['camera_matrix']).reshape(3, 3)
        d = np.asarray(calibration['distort_coeffs'])
        corners = cv.projectPoints(obj, cv.Rodrigues(cr)[0], ct, k, d)[0].reshape(4, 2).astype(np.float32)
        ok, rv, tv = cv.solvePnP(obj, corners, k, d, flags=cv.SOLVEPNP_IPPE)
        self.assertTrue(ok)
        replay_xyz = rot @ (rcg @ tv.reshape(3) + calibration['t_camera2gimbal'])
        raw = rot @ rcg @ cv.Rodrigues(rv)[0]
        observation = {'face_id': 0, 'type': 1, 'name': 3, 'corners_px': corners.tolist(),
                       'xyz_world_m': replay_xyz.tolist(), 'body_yaw_rad': yaw,
                       'pnp_yaw_before_optimization_rad': math.atan2(raw[1, 0], raw[0, 0])}
        frame = {'run': 1, 'frame_seq': 0, 'frame_imu_valid': True, 'tracker_converged': True,
                 'solver_R_gimbal2world_rowmajor': rot.flatten().tolist(), 'observations': [observation]}
        return frame, observation, calibration

    def test_nontrivial_extrinsics_replay_and_planar_candidates(self):
        f, o, c = self.synthetic()
        row = MODULE.replay_pnp_observation(f, o, c, self.cv, self.np)
        self.assertAlmostEqual(row['replay_xyz_difference_cm'], 0., places=8)
        self.assertAlmostEqual(row['replay_raw_yaw_difference_deg'], 0., places=8)
        self.assertAlmostEqual(row['replay_optimized_yaw_difference_deg'], 0., places=5)
        self.assertLess(row['ippe_reprojection_rms_px'], .001)
        self.assertIn('ippe_solution_yaw_separation_deg', row)

    def test_missing_corners_and_conflicts_are_not_guessed(self):
        f, o, c = self.synthetic()
        del o['corners_px']
        report = MODULE.analyze_pnp_replay([f], {1: c}, set(), 1)
        self.assertEqual(report['exclusions']['missing_or_invalid_corners'], 1)
        report = MODULE.analyze_pnp_replay([f], {1: c}, {1}, 1)
        self.assertEqual(report['exclusions']['conflicting_calibration'], 1)

    def test_bad_transform_and_balance_branch(self):
        f, o, c = self.synthetic()
        f['solver_R_gimbal2world_rowmajor'] = [0.] * 9
        with self.assertRaisesRegex(ValueError, 'non_rotation_matrix'):
            MODULE.replay_pnp_observation(f, o, c, self.cv, self.np)
        f, o, c = self.synthetic()
        o.update(type=0, name=3, pnp_yaw_before_optimization_rad=None)
        row = MODULE.replay_pnp_observation(f, o, c, self.cv, self.np)
        self.assertNotIn('replay_optimized_yaw_difference_deg', row)

    @unittest.skipUnless(Path('/usr/include/opencv4').is_dir() and shutil.which('g++'), 'Linux C++ offline dependencies required')
    def test_actual_cpp_solver_output_replays(self):
        root = Path(__file__).resolve().parents[1]
        harness = r'''
#include <iostream>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include "tasks/auto_aim/solver.hpp"
int main(int argc, char **argv) {
  auto cfg=YAML::LoadFile(argv[1]); auto_aim::Solver solver(argv[1]);
  for(double gimbal : {-.12, .04, .15})
  for(double yaw : {-.65, .05, .65})
  for(bool balance : {false, true}) {
    auto q=Eigen::AngleAxisd(gimbal,Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-.03,Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(.01,Eigen::Vector3d::UnitX());
    solver.set_R_gimbal2world(Eigen::Quaterniond(q));
    auto_aim::Armor armor(11,.9f,cv::Rect(0,0,100,40),{{0,0},{100,0},{100,40},{0,40}});
    armor.type=balance?auto_aim::ArmorType::big:auto_aim::ArmorType::small;
    armor.name=auto_aim::ArmorName::three;
    armor.points=solver.reproject_armor({3.8,.1,.03},yaw,armor.type,armor.name);
    solver.solve(armor);
    nlohmann::json c={{"solver_model","ippe_fixed_pitch_yaw_grid_v1"}};
    for(const char* key : {"camera_matrix","distort_coeffs","R_camera2gimbal","t_camera2gimbal"})
      c[key]=cfg[key].as<std::vector<double>>();
    nlohmann::json f={{"solver_R_gimbal2world_rowmajor",nlohmann::json::array()}};
    auto r=solver.R_gimbal2world();
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) f["solver_R_gimbal2world_rowmajor"].push_back(r(i,j));
    nlohmann::json o={{"type",int(armor.type)},{"name",int(armor.name)},
      {"xyz_world_m",{armor.xyz_in_world[0],armor.xyz_in_world[1],armor.xyz_in_world[2]}},
      {"body_yaw_rad",armor.ypr_in_world[0]}, {"corners_px",nlohmann::json::array()},
      {"pnp_yaw_before_optimization_rad",nullptr}};
    if(!balance) o["pnp_yaw_before_optimization_rad"]=armor.yaw_raw;
    for(const auto & p:armor.points) o["corners_px"].push_back({p.x,p.y});
    std::cout << nlohmann::json({{"frame",f},{"observation",o},{"calibration",c}}).dump() << "\n";
  }
}
'''
        binary = root / 'build/solver_prediction_replay_test'
        binary.parent.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='solver-replay-test-') as folder:
            source = Path(folder) / 'test.cpp'
            source.write_text(harness, encoding='utf-8')
            subprocess.run(['g++', '-std=c++17', '-O1', '-pthread', '-I' + str(root),
                            '-I/usr/include/eigen3', '-I/usr/include/opencv4', str(source),
                            *[str(root / name) for name in ['tasks/auto_aim/solver.cpp', 'tasks/auto_aim/armor.cpp',
                                                            'tools/math_tools.cpp', 'tools/logger.cpp']],
                            '-lopencv_core', '-lopencv_calib3d', '-lopencv_imgproc', '-lyaml-cpp', '-lspdlog', '-lfmt',
                            '-o', str(binary)], check=True, timeout=180, capture_output=True)
            result = subprocess.run([str(binary), str(root / 'configs/standard.yaml')], check=True,
                                    capture_output=True, text=True, timeout=30)
        examples = [json.loads(line) for line in result.stdout.splitlines() if line.startswith('{')]
        self.assertEqual(len(examples), 18)
        for item in examples:
            row = MODULE.replay_pnp_observation(item['frame'], item['observation'], item['calibration'], self.cv, self.np)
            self.assertLess(row['replay_xyz_difference_cm'], 1e-4)
            self.assertLess(abs(row.get('replay_raw_yaw_difference_deg', 0.)), 1e-4)
            self.assertLess(abs(row.get('replay_optimized_yaw_difference_deg', 0.)), 1e-4)


if __name__ == "__main__":
    unittest.main()
