"""Offline causality tests for the diagnostic candidate; no hardware needed."""
from copy import deepcopy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from analyze_high_speed_rotation import MaturedFaceReview, causal_candidate
from analyze_prediction_debug import analyze_camera_policy, analyze_yaw_zero_shadow, compare_yaw_zero_references


class ZeroShadowReviewTests(unittest.TestCase):
    @staticmethod
    def recorded(seq, error=0., **updates):
        f = {"run": 1, "frame_seq": seq, "rotation_mode": "high_speed",
             "gate_prediction_source": "native_prediction", "fire_branch": "vehicle_prediction",
             "native_prediction_valid": True, "gate_error_rad": math.radians(error),
             "gate_tolerance_rad": math.radians(.2), "yaw_boresight_correction_rad": math.radians(-1.3),
             "yaw_impact_correction_rad": 0., "request_shoot": seq == 1, "omega_rad_s": -4.5}
        f.update(updates)
        return f

    def test_same_original_frames_without_config_or_future_labels(self):
        rows = [self.recorded(1), self.recorded(2, 1.3), self.recorded(3, .7)]
        before = deepcopy(rows)
        result = analyze_yaw_zero_shadow(rows)
        counts = result["all_compared"]
        self.assertEqual((counts["frames"], counts["legacy_only_pass"],
                          counts["command_frame_only_pass"], counts["neither_pass"]), (3, 1, 1, 1))
        self.assertEqual(result["same_original_requests"]["legacy_only_pass"], 1)
        self.assertFalse(result["physical_bore_zero_verified"])
        self.assertFalse(result["new_requests_simulated"])
        self.assertEqual(rows, before)
        later = self.recorded(4, -120., native_prediction_ns=999999999999,
                              future_error_deg=-10000.)
        self.assertEqual(result["samples"], analyze_yaw_zero_shadow(rows + [later])["samples"][:3])

    def test_modes_invalid_and_unrecorded_offsets_are_excluded(self):
        rows = [self.recorded(1, rotation_mode="low_speed"),
                self.recorded(2, rotation_mode="stationary"),
                self.recorded(3, gate_prediction_source="pll_event"),
                self.recorded(4, gate_error_rad=None),
                self.recorded(5, yaw_boresight_correction_rad=None)]
        result = analyze_yaw_zero_shadow(rows)
        self.assertEqual(result["all_compared"]["frames"], 0)
        self.assertEqual(sum(result["exclusions"].values()), 3)

    def test_fixed_center_source_uses_its_recorded_gate(self):
        f = self.recorded(1, gate_prediction_source="fixed_center_prediction")
        result = analyze_yaw_zero_shadow([f])
        self.assertEqual(result["same_original_requests"]["frames"], 1)
        self.assertEqual(result["samples"][0]["prediction_source"], "fixed_center_prediction")

    def test_runtime_shadow_mismatch_is_not_silently_accepted(self):
        f = self.recorded(1)
        f["yaw_zero_shadow"] = {"valid": True,
                                "command_correction_rad": math.radians(-1.3),
                                **compare_yaw_zero_references(0., math.radians(-1.3), math.radians(.2))}
        self.assertEqual(analyze_yaw_zero_shadow([f])["all_compared"]["frames"], 1)
        f["yaw_zero_shadow"]["command_frame_error_rad"] += .01
        self.assertEqual(analyze_yaw_zero_shadow([f])["exclusions"]["runtime_shadow_mismatch"], 1)

    def test_wrap_and_inclusive_boundary(self):
        for error in (-.2, .2):
            r = compare_yaw_zero_references(math.radians(error), 0., math.radians(.2))
            self.assertTrue(r["legacy_pass"] and r["command_frame_pass"])
        self.assertAlmostEqual(compare_yaw_zero_references(
            math.radians(179.9), math.radians(.2), 1.)["command_frame_error_rad"], math.radians(-179.9))
        self.assertIsNone(compare_yaw_zero_references(float("nan"), 0., .2))
        self.assertIsNone(compare_yaw_zero_references(0., 0., -.2))

    def test_camera_metadata_never_invents_old_log_age(self):
        self.assertEqual(analyze_camera_policy([self.recorded(1)])["status"], "no_capture_metadata")
        result = analyze_camera_policy([
            self.recorded(1, camera_capture_sequence=4, camera_latest_selected=True,
                          camera_age_at_read_ms=3., camera_skipped_frames=2, frame_imu_valid=False),
            self.recorded(2, camera_capture_sequence=5, camera_latest_selected=False,
                          camera_age_at_read_ms=12., frame_imu_valid=True)])
        self.assertEqual(result["groups"]["latest"]["imu_invalid"], 1)
        self.assertEqual(result["groups"]["latest"]["skipped_converted_frames"], 2)
        self.assertEqual(result["groups"]["first_pending"]["age_at_read_ms"]["mean"], 12.)


def frame(seq, time_s, *, epoch=1, face=0, omega=-4.5, mode="vehicle", prediction_s=None):
    return {"run": 1, "frame_seq": seq, "frame_ns": round(time_s * 1e9),
            "native_prediction_ns": round((time_s + .5 if prediction_s is None else prediction_s) * 1e9),
            "native_face_id": face, "track_epoch": epoch, "omega_rad_s": omega,
            "motion_mode": mode, "request_shoot": False}


def residual(f, error):
    return {"run": f["run"], "frame_seq": f["frame_seq"],
            "face_id": f["native_face_id"], "yaw_error_deg": error}


class MaturedReviewTests(unittest.TestCase):
    def test_adjacent_frames_do_not_supply_independent_confirmation(self):
        m = MaturedFaceReview()
        for i in range(50):
            m.observe_matured((1, 1, 0), i * 1_000_000, -.6)
        m.advance(400_000_000)
        self.assertEqual(len(m.cycles[(1, 1, 0)]), 1)
        self.assertEqual(m.correction((1, 1, 0)), 0)
        for at in (1_000_000_000, 2_000_000_000):
            m.observe_matured((1, 1, 0), at, -.6)
            m.advance(at + 300_000_000)
        self.assertAlmostEqual(m.correction((1, 1, 0)), .3)

    def test_sign_cap_expiry_and_identity_isolation(self):
        m = MaturedFaceReview()
        for seconds in (1, 2, 3):
            for face, error in ((0, -.8), (1, .7)):
                m.observe_matured((1, 1, face), seconds * 1_000_000_000, error)
            m.advance(seconds * 1_000_000_000 + 300_000_000)
        self.assertEqual(m.correction((1, 1, 0)), .3)
        self.assertEqual(m.correction((1, 1, 1)), -.3)
        self.assertEqual(m.correction((1, 2, 0)), 0)
        self.assertEqual(m.correction((2, 1, 0)), 0)
        m.advance(12_000_000_000)
        self.assertEqual(m.correction((1, 1, 0)), 0)

    def test_future_reference_cannot_change_earlier_queries(self):
        fs = [frame(i, i * .05) for i in range(120)]
        rs = [residual(fs[i], -.5) for i in (0, 20, 40, 80)]
        original = causal_candidate(fs, rs)
        changed = deepcopy(rs)
        changed[-1]["yaw_error_deg"] = .9
        other = causal_candidate(fs, changed)
        # The changed label belongs to t=4s, predicted for 4.5s. Its entire
        # interpolation bracket remains unavailable until 4.525s.
        for f in fs:
            if f["frame_ns"] < 4_525_000_000:
                self.assertEqual(original["corrections"][(1, f["frame_seq"])],
                                 other["corrections"][(1, f["frame_seq"])])
        self.assertGreater(original["corrections"][(1, 70)]["online_deg"], 0)

    def test_frozen_second_half_never_learns_second_half_labels(self):
        fs = [frame(i, i * .1) for i in range(121)]
        rs = [residual(fs[i], -.6) for i in (0, 10, 20, 40, 80, 90, 100)]
        original = causal_candidate(fs, rs)
        changed = deepcopy(rs)
        for r in changed:
            if r["frame_seq"] >= 80:
                r["yaw_error_deg"] = .8
        other = causal_candidate(fs, changed)
        for f in fs[60:]:
            self.assertEqual(original["corrections"][(1, f["frame_seq"])]["frozen_deg"],
                             other["corrections"][(1, f["frame_seq"])]["frozen_deg"])
        self.assertEqual(original["corrections"][(1, 100)]["frozen_deg"], .3)
        self.assertNotEqual(original["corrections"][(1, 120)]["online_deg"],
                            other["corrections"][(1, 120)]["online_deg"])

    def test_low_speed_and_new_target_do_not_inherit_correction(self):
        fs = [frame(i, i * .1) for i in range(70)]
        fs[40] = frame(40, 4, omega=-1.6)
        fs[41] = frame(41, 4.1, epoch=2)
        fs[42] = frame(42, 4.2, mode="armor")
        fs[43] = frame(43, 4.3, omega=4.5)
        result = causal_candidate(fs, [residual(fs[i], -.6) for i in (0, 10, 20)])
        self.assertEqual(result["corrections"][(1, 39)]["online_deg"], .3)
        for seq in (40, 41, 42, 43):
            self.assertEqual(result["corrections"][(1, seq)]["online_deg"], 0)

    def test_reject_out_of_order_residuals(self):
        m = MaturedFaceReview()
        m.observe_matured((1, 1, 0), 2, .1)
        with self.assertRaises(ValueError):
            m.observe_matured((1, 1, 0), 1, .1)


if __name__ == "__main__":
    unittest.main()
