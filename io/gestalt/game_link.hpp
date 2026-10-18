// GameIO backend: WebSocket link to the gestalt UE game.
// Replaces io::CBoard for the bridge: telemetry cache (attribute.watchAttributeMaps,
// ~30Hz turret pose push) + console.exec sender (UEExec RBExtAim / Respawn /
// BatchSet ...). Mirrors tools/game_window_capture.py GameLink semantics,
// including the ptr-map (attr 1000001) auto-discovery and reconnect-or-freeze
// behaviour ("without reconnect the telemetry cache FREEZES at stale values").
#ifndef IO_GESTALT_GAME_LINK_HPP
#define IO_GESTALT_GAME_LINK_HPP

#include <atomic>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace io::gestalt
{
// Attribute ids used by the bridge (see tools/*.py and Monitor attributes.ts).
namespace attr
{
constexpr int kPlayerId = 10000035;
constexpr int kTeamId = 10000036;
constexpr int kConnectionEntityConfigId = 10000064;
constexpr int kClass = 60000002;
constexpr int kPosX = 10000107, kPosY = 10000108, kPosZ = 10000109;
constexpr int kChassisYaw = 10000110;
constexpr int kTurretYaw = 10000111, kTurretPitch = 10000112;
constexpr int kHealth = 10000003, kHealthMax = 60000004;
constexpr int kWeakened = 50000002, kDefeated = 50000007;
constexpr int kIsChassisOnline = 50000014, kCanOperate = 50000021;
constexpr int kAllowance17mm = 10000033, kReal17mm = 10000031;
constexpr int kBulletsFired = 63000002;
constexpr int kDamageApplied = 63000000;   // cumulative damage dealt by the vehicle
constexpr int kReviveProgress = 10000022;  // fills toward kReviveProgressMax while dead
constexpr int kReviveProgressMax = 60000017;
constexpr int kShooterRealSpeed = 10000030;  // per-shot muzzle speed, cm/s
constexpr int kIsAI = 50000088, kMoveMode = 50000089, kTargetMode = 50000090;
constexpr int kMoveMarker = 50000097;
constexpr int kMatchStatus = 80000005;
constexpr int kMapPtr = 1000001;
constexpr int kOutpostRedGlobal = 80002000;
}  // namespace attr

class GameLink
{
public:
  explicit GameLink(int port);
  ~GameLink();
  GameLink(const GameLink &) = delete;
  GameLink & operator=(const GameLink &) = delete;

  bool connected() const { return connected_.load(std::memory_order_acquire); }
  // Monotonically increases after every successful WebSocket connection.
  // A caller can therefore distinguish a brief disconnect/reconnect from an
  // uninterrupted session even when it never samples connected()==false.
  uint64_t connection_generation() const
  {
    return connection_generation_.load(std::memory_order_acquire);
  }

  // Latest cached attribute value (numeric attrs only). nullopt if unseen.
  std::optional<double> attr(int map_id, int attr_id) const;

  // Console command through the game's TS console (fire-and-forget).
  void exec(const std::string & command);
  // Sends only when the socket is still the exact connection generation the
  // caller fixed. The identity check and WebSocket send share ws_mu_, so a
  // reconnect cannot redirect a lease renewal or gate command to a new socket.
  bool exec_if_generation(const std::string & command, uint64_t expected_generation);

  // rgbCamera.applySettings with a raw JSON camera object, e.g.
  // R"({"enabled":1,"fovDegrees":45,"shutterSpeed":120})".
  void apply_camera(const std::string & camera_json);
  bool apply_camera_if_generation(
    const std::string & camera_json, uint64_t expected_generation);

  // Pose history for one designated vehicle map: every turret yaw/pitch push
  // is timestamped on arrival so callers can interpolate the pose TO THE FRAME
  // INSTANT (standard.cpp slerps IMU to image time; the 30Hz telemetry
  // otherwise runs up to ~33ms out of step with the captured pixels, injecting
  // false angular motion into the EKF during slews).
  void enable_pose_history(int map_id);
  // Interpolated (yaw_deg, pitch_deg) at steady-clock time t_s; falls back to
  // the nearest sample when not bracketed (extrapolation clamped to 60ms).
  std::optional<std::pair<double, double>> pose_at(double t_s) const;

  // First map id matching class/team (e.g. blue sentry: cls 1004, team 1),
  // preferring maps whose kHealth > 0 to dodge stale corpse maps from earlier
  // Respawns (the "dead sentry frozen snapshot" trap).
  std::optional<int> find_vehicle(int cls, int team) const;

  // Newest live combat map owned by an exact player id. Test scenes may have
  // multiple same-team/same-class vehicles, so control/camera identity must
  // never be inferred from class filters.
  std::optional<int> find_player(int player_id) const;

  // Entity config selected for a player (for example pid 0 -> 66000005 for
  // HACHISEN). This lives on the player attribute map, not the combat map.
  std::optional<int> player_entity_config(int player_id) const;

  // Map id of the red outpost (global var kOutpostRedGlobal, fallback: the
  // 1500-HP building map without a player id).
  std::optional<int> find_red_outpost() const;

  double match_status() const;  // -1 unknown, 0 prep, 1 running, 2 settled

private:
  void reader_loop();
  void watch(const std::vector<int> & ids);
  void send_raw(const std::string & payload);
  bool send_raw_if_generation(const std::string & payload, uint64_t expected_generation);
  bool connect_once();

  struct PoseSample
  {
    double t;    // steady-clock seconds at arrival
    double yaw;  // deg, world
    double pitch;
  };

  int port_;
  std::atomic<bool> stop_{false}, connected_{false};
  std::atomic<uint64_t> connection_generation_{0};
  std::atomic<int> pose_map_{-1};
  mutable std::mutex mu_;                     // guards maps_/watched_/pose_hist_
  mutable std::mutex ws_mu_;                  // serializes easywsclient poll/dispatch/send
  std::deque<PoseSample> pose_hist_;
  std::map<int, std::map<int, double>> maps_; // map_id -> attr_id -> value
  std::set<int> watched_;
  // Requests are emitted by both the main/control thread and reader-thread
  // discovery path. Keep ids unique without relying on the websocket mutex:
  // JSON is assembled before send_raw() takes that lock.
  std::atomic<int> next_id_{990001};
  void * ws_ = nullptr;  // easywsclient::WebSocket*
  std::thread reader_;
};

}  // namespace io::gestalt

#endif  // IO_GESTALT_GAME_LINK_HPP
