#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

#include <cstdint>

namespace io
{
struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;  //无人机专有
  bool inhibit_shoot = false;
  bool request_return = false;  // XUC: explicit control_TJ=2, latched by the sender.
  int64_t target_valid_until_ns = 0; // Steady clock; zero uses the sender's bounded freshness lease.
};

}  // namespace io

#endif  // IO__COMMAND_HPP
