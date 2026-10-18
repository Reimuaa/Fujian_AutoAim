// Windows no-op shim for Linux <sched.h>.
//
// tasks/auto_aim/yolos/{yolo11,yolo26}.cpp pin the inference thread to the
// big cores with sched_setaffinity(2), a Linux-only syscall. On the Gestalt
// simulator build the model runs on CPU with the OpenVINO thread pool already
// capped, so affinity binding is a no-op. This directory is on the include
// path for the Windows build ONLY, keeping the upstream 09-12 sources
// byte-identical (same approach as the io/socketcan.hpp in-file stub).
#ifndef WIN_SHIMS__SCHED_H
#define WIN_SHIMS__SCHED_H

#include <cstddef>

using cpu_set_t = unsigned long;

inline void CPU_ZERO(cpu_set_t * set)
{
  if (set != nullptr) *set = 0;
}

inline void CPU_SET(int cpu, cpu_set_t * set)
{
  if (set != nullptr && cpu >= 0 &&
      cpu < static_cast<int>(sizeof(cpu_set_t) * 8)) {
    *set |= (1UL << cpu);
  }
}

// Binding is unsupported on the simulator; report "failure" so the caller
// silently keeps the OS-default affinity (the callers only log on success).
inline int sched_setaffinity(int, std::size_t, const cpu_set_t *)
{
  return -1;
}

#endif  // WIN_SHIMS__SCHED_H
