"""Offline boundary/causality tests; no hardware, logs, or third-party packages."""
import copy
import math
from pathlib import Path
import sys
import shutil
import subprocess
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from replay_rotation_window import decide, exiting_next_step, finite, phase_relative_to_center, read_frames

ROOT = Path(__file__).resolve().parents[1]


def frame(seq, phase_deg, omega=1.8):
    return {
        "frame_seq": seq, "frame_ns": seq * 20_000_000,
        "aimer_call_end_ns": seq * 20_000_000 + 5_000_000,
        "native_prediction_ns": seq * 20_000_000 + 500_000_000,
        "native_prediction_valid": True, "native_geometry_source": "whole_ekf_rigid",
        "whole_ekf_state_at_frame": [3, 0, 0, 0, 0, 0, 0, omega, .3, 0, 0],
        "native_xyza": [3, 0, 0, math.radians(phase_deg)], "track_epoch": 1,
        "track_epoch_reset": False, "target_name": 3, "target_type": 1,
        "armor_count": 4, "native_face_id": 0, "fire_branch": "vehicle_prediction",
        "fire_gates": {"auto_mode": True, "control": True, "yaw_error": True,
                       "command_rearm": True, "stable_window": True},
        "rotation_fire_window": {"current_frame_is_better": False},
    }


class BoundaryReplayTest(unittest.TestCase):
    def test_both_rotation_directions(self):
        for sign in (1, -1):
            self.assertTrue(exiting_next_step(frame(1, sign * 18, sign * 1.8),
                                             frame(2, sign * 19.2, sign * 1.8), 60, 20))

    def test_interior_and_retreat_do_not_release(self):
        self.assertFalse(exiting_next_step(frame(1, 4), frame(2, 5), 60, 20))
        self.assertFalse(exiting_next_step(frame(1, 19.8), frame(2, 19), 60, 20))

    def test_identity_discontinuity_cannot_release(self):
        a, b = frame(1, 18), frame(2, 19.2)
        for key, value in [("track_epoch", 2), ("target_name", 2), ("target_type", 0),
                           ("native_face_id", 1), ("track_epoch_reset", True),
                           ("frame_seq", 3), ("native_prediction_valid", False)]:
            changed = copy.deepcopy(b)
            changed[key] = value
            self.assertFalse(exiting_next_step(a, changed, 60, 20), key)

    def test_existing_gates_are_never_bypassed(self):
        a, b = frame(1, 18), frame(2, 19.2)
        for gate in b["fire_gates"]:
            changed = copy.deepcopy(b)
            changed["fire_gates"][gate] = False
            self.assertEqual(decide([a, changed]), [], gate)

    def test_cooldown_blocks_nearby_second_request(self):
        rows = [frame(1, 18), frame(2, 19.2), frame(3, 19.8)]
        self.assertEqual([r["frame_seq"] for r in decide(rows)], [2])

    def test_future_rows_do_not_change_earlier_decision(self):
        prefix = [frame(1, 18), frame(2, 19.2)]
        for valid in (True, False):
            future = frame(3, 21)
            future["native_prediction_valid"] = valid
            result = [r["frame_seq"] for r in decide(prefix + [future]) if r["frame_seq"] <= 2]
            self.assertEqual(result, [2])


@unittest.skipUnless(sys.platform == "linux" and shutil.which("g++"), "Linux C++ compiler required")
class RuntimeBoundaryTest(unittest.TestCase):
    """Compile the production window and request expression, without device I/O."""

    @classmethod
    def setUpClass(cls):
        source = (ROOT / "src/standard.cpp").read_text(encoding="utf-8")
        start = source.index("struct RotationFireWindow")
        window = source[start:source.index("\n};", start) + 3]
        start = source.index("        command.shoot = stable_window_open && fire_candidate &&")
        request = source[start:source.index(";", start) + 1]
        harness = r'''
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
WINDOW
int main() {
  RotationFireWindow rotation_fire_window;
  uint64_t seq;
  int64_t ns, request_ns, last_request_ns = std::numeric_limits<int64_t>::min() / 2;
  int valid, rebuilt, name, type, count, face, event, selected, candidate, yaw;
  double phase, error, relative, omega;
  while (std::cin >> seq >> ns >> valid >> rebuilt >> name >> type >> count >> face
         >> event >> phase >> error >> selected >> relative >> omega >> candidate >> yaw
         >> request_ns) {
    rotation_fire_window.update(valid, rebuilt, name, type, count, face, event, phase, error,
      std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ns)), 2,
      25.0 * M_PI / 180.0, 0.6 * M_PI / 180.0, seq,
      selected ? relative : NAN, selected ? omega : NAN, 60.0 / 57.3, 20.0 / 57.3);
    const bool stable_window_open = rotation_fire_window.two_frames_ready;
    // Frozen logs retain the original sender state. Model the unchanged
    // 100/50 ms minimum cadence after an additional simulated request, as decide() does.
    const bool fire_candidate = candidate && request_ns - last_request_ns >= 150000000;
    const bool prediction_yaw_gate_open = yaw;
    const bool prediction_point_gate_open = rotation_fire_window.current_frame_is_better ||
      rotation_fire_window.exiting_selection_next_step;
    struct { bool shoot = false; } command;
REQUEST
    if (command.shoot) last_request_ns = request_ns;
    std::cout << rotation_fire_window.current_frame_is_better << ' '
              << rotation_fire_window.exiting_selection_next_step << ' ' << command.shoot << '\n';
  }
}
'''.replace("WINDOW", window).replace("REQUEST", request)
        cls.binary = ROOT / "build/test_rotation_window_boundary"
        cls.binary.parent.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="rotation-window-") as directory:
            cpp = Path(directory) / "test.cpp"
            cpp.write_text(harness, encoding="utf-8")
            subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                            str(cpp), "-o", str(cls.binary)], check=True, timeout=60)

    def run_runtime(self, rows):
        lines, previous = [], None
        for row in rows:
            gates = row.get("fire_gates", {})
            phase = phase_relative_to_center(row)
            state = row.get("whole_ekf_state_at_frame") or []
            point = row.get("native_xyza") or []
            error = row.get("gate_error_rad", math.radians(0.5))
            valid = bool(row.get("native_prediction_valid") and len(point) == 4 and finite(error)
                         and all(gates.get(k, True) for k in
                                 ("target", "tracker_converged", "yaw_observation",
                                  "pitch_observation", "frame_imu")))
            rebuilt = bool(row.get("track_epoch_reset") or
                           previous and row["track_epoch"] != previous["track_epoch"])
            values = [row["frame_seq"], row["frame_ns"], int(valid), int(rebuilt),
                      row["target_name"], row["target_type"], row["armor_count"],
                      row["native_face_id"], int(row["fire_branch"] == "high_speed_event"),
                      point[3] if point else 0, error if finite(error) else 0,
                      int(phase is not None), phase if phase is not None else 0,
                      state[7] if len(state) == 11 else 0,
                      int(bool(gates) and all(gates.values())), int(gates.get("yaw_error", False)),
                      row["aimer_call_end_ns"]]
            lines.append(" ".join(map(str, values)))
            previous = row
        result = subprocess.run([str(self.binary)], input="\n".join(lines) + "\n",
                                text=True, capture_output=True, check=True, timeout=30)
        decisions = [tuple(map(int, line.split())) for line in result.stdout.splitlines()]
        self.assertEqual(len(decisions), len(rows))
        return decisions

    def test_runtime_boundary_and_all_existing_gates(self):
        for sign in (1, -1):
            a, b = frame(1, sign * 18, sign * 1.8), frame(2, sign * 19.2, sign * 1.8)
            a["gate_error_rad"], b["gate_error_rad"] = math.radians(.6), math.radians(.5)
            self.assertEqual(self.run_runtime([a, b])[-1], (0, 1, 1))
            for gate in b["fire_gates"]:
                changed = copy.deepcopy(b)
                changed["fire_gates"][gate] = False
                self.assertEqual(self.run_runtime([a, changed])[-1][-1], 0, gate)

    def test_runtime_discontinuity_and_other_modes(self):
        a, b = frame(1, 18), frame(2, 19.2)
        for key, value in [("frame_seq", 3), ("track_epoch", 2), ("track_epoch_reset", True),
                           ("native_face_id", 1), ("target_name", 2), ("target_type", 0),
                           ("armor_count", 3), ("native_geometry_source", "existing"),
                           ("fire_branch", "high_speed_event")]:
            changed = copy.deepcopy(b)
            changed[key] = value
            self.assertEqual(self.run_runtime([a, changed])[-1][1], 0, key)
        changed = copy.deepcopy(b)
        changed["whole_ekf_state_at_frame"][7] *= -1
        self.assertEqual(self.run_runtime([a, changed])[-1][1], 0)

    def test_runtime_matches_both_recorded_replays(self):
        for name, expected in [("2026-10-17_17-33-13.log", (11, 55)),
                               ("2026-10-17_16-01-30.log", (87, 93))]:
            path = ROOT / "logs" / name
            if not path.exists():
                self.skipTest("Recorded field logs are not available")
            rows = read_frames(path)
            decisions = self.run_runtime(rows)
            previous = None
            for row, (_, boundary, _) in zip(rows, decisions):
                if row.get("fire_branch") == "vehicle_prediction" and all(row["fire_gates"].values()):
                    self.assertEqual(bool(boundary), exiting_next_step(previous, row, 60, 20))
                previous = row
            selected = [r for r, decision in zip(rows, decisions)
                        if r["fire_branch"] == "vehicle_prediction" and decision[-1]]
            baseline = [r["frame_seq"] for r in rows
                        if r["fire_branch"] == "vehicle_prediction" and r["request_shoot"]]
            self.assertEqual((len(baseline), len(selected)), expected)
            self.assertEqual([r["frame_seq"] for r in selected],
                             [r["frame_seq"] for r in decide(rows)])
            self.assertTrue(set(baseline).issubset(r["frame_seq"] for r in selected))


if __name__ == "__main__":
    unittest.main()
