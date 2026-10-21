"""Offline causality tests for the diagnostic candidate; no hardware needed."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from analyze_high_speed_rotation import MaturedFaceReview, causal_candidate


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
