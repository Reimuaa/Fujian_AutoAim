// Local, frame-atomic UE vision stream. The game publishes the final viewport
// backbuffer and the exact FSceneView pose that rendered it in one shared-memory
// slot. No AttributeMap/TS/network telemetry participates in this path.
#ifndef IO_GESTALT_SHARED_FRAME_CAPTURE_HPP
#define IO_GESTALT_SHARED_FRAME_CAPTURE_HPP

#include "win_capture.hpp"

#include <cstdint>
#include <functional>
#include <string>

namespace io::gestalt
{
class SharedFrameCapture
{
public:
  SharedFrameCapture();
  ~SharedFrameCapture();

  SharedFrameCapture(const SharedFrameCapture &) = delete;
  SharedFrameCapture & operator=(const SharedFrameCapture &) = delete;

  // Resolves the process listening on the loopback WebSocket port, finds that
  // exact process' game window, and opens only its process-scoped mapping.
  // This prevents a second UE publisher from stealing the pixel data plane.
  // An optional interruption callback makes the mapping wait cancellable for
  // match settlement or WebSocket-generation loss.
  bool init(
    int ws_port, int timeout_ms = 10000,
    const std::function<bool()> & interrupted = {});
  CapturedFrame grab(int timeout_ms = 100);
  void raise_window();

  int full_client_height() const { return height_; }
  int width() const { return width_; }
  int height() const { return height_; }
  std::string window_title() const { return title_; }
  uint32_t process_id() const { return process_id_; }

private:
  struct Impl;
  Impl * impl_;
  int width_ = 0;
  int height_ = 0;
  std::string title_;
  uint32_t process_id_ = 0;
  uint64_t last_commit_ = 0;
};
}  // namespace io::gestalt

#endif
