#pragma once

// Raw frames straight out of the ISP, for consumers other than the JPEG still
// path - an H.264 encoder, say. Kept free of anything ESPHome- or
// camera-specific so a consumer only needs this header.

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace p4_csi_camera {

/// One frame as the ISP delivered it. Valid only for the duration of
/// on_raw_frame(): the buffer goes straight back to the capture hardware when
/// the call returns, so a consumer that needs it later must copy or encode
/// before returning.
struct RawFrame {
  const uint8_t *data;
  size_t len;
  uint16_t width;
  uint16_t height;
  /// V4L2 four-character pixel format code, e.g. 'YU12' for YUV 4:2:0.
  uint32_t fourcc;
  /// Counts up by one per frame the capture task saw; a gap means a frame
  /// was not delivered to sinks.
  uint32_t sequence;
  /// esp_timer_get_time() when the frame was dequeued.
  int64_t timestamp_us;
};

/// Receives raw frames on the camera's capture task, at the sensor's rate,
/// while streaming has been asked for. Must not block for long: the sensor
/// keeps producing, and a frame that is not collected in time is dropped.
class RawVideoSink {
 public:
  virtual ~RawVideoSink() = default;
  virtual void on_raw_frame(const RawFrame &frame) = 0;
};

}  // namespace p4_csi_camera
}  // namespace esphome
