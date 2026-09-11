#pragma once

// The panel's built-in camera, over MIPI-CSI, as an ESPHome camera entity.
//
// The sensor is an OV02C10 at 0x36 on the same I2C bus as the GT911 touch
// controller - confirmed by reading its chip id from the device (0x300A..0x300C
// = 0x56 0x02 0x43). Its driver is idf/ov02c10. Underneath is Espressif's
// esp_video: the CSI controller and ISP appear as /dev/video0, the hardware
// JPEG encoder as /dev/video10.
//
// Shape of the thing: the main loop only ever sets request flags and hands
// finished JPEGs to the API. A dedicated task owns the V4L2 session - opens it
// on the first request, pulls frames, encodes them, and closes it again a few
// seconds after the last request - because V4L2 dequeues block and ESPHome's
// main loop is watchdogged.

#include "esphome/core/defines.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

#include <atomic>
#include <memory>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <driver/ppa.h>

#include "esphome/components/camera/camera.h"
#include "raw_video.h"
#include "esphome/components/i2c/i2c_bus.h"
#include "esphome/core/component.h"

namespace esphome {
namespace p4_csi_camera {

/// Index into the sensor driver's format table (idf/ov02c10/ov02c10.c).
enum SensorMode : uint8_t {
  SENSOR_MODE_1288X728 = 0,   // one MIPI lane
  SENSOR_MODE_1920X1080 = 2,  // two MIPI lanes, the driver's default
};

/// One encoded frame. The bytes are a heap copy owned by this object and freed
/// with the last reference, so the encoder's buffer can be reused immediately.
class JpegImage : public camera::CameraImage {
 public:
  JpegImage(uint8_t *data, size_t len, uint16_t width, uint16_t height, uint8_t requesters)
      : data_(data), len_(len), width_(width), height_(height), requesters_(requesters) {}
  ~JpegImage() override;
  uint8_t *get_data_buffer() override { return this->data_; }
  size_t get_data_length() override { return this->len_; }
  bool was_requested_by(camera::CameraRequester requester) const override {
    return (this->requesters_ & (1u << requester)) != 0;
  }
  uint16_t get_width() const { return this->width_; }
  uint16_t get_height() const { return this->height_; }

 protected:
  uint8_t *data_;
  size_t len_;
  uint16_t width_;
  uint16_t height_;
  uint8_t requesters_;
};

/// The API's cursor into a JpegImage; one per connection.
class JpegImageReader : public camera::CameraImageReader {
 public:
  void set_image(std::shared_ptr<camera::CameraImage> image) override;
  size_t available() const override;
  uint8_t *peek_data_buffer() override;
  void consume_data(size_t consumed) override;
  void return_image() override;

 protected:
  std::shared_ptr<camera::CameraImage> image_;
  size_t offset_{0};
};

class P4CsiCamera : public camera::Camera {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  // After i2c, whose bus handle this borrows.
  float get_setup_priority() const override { return setup_priority::LATE; }

  // camera::Camera
  void add_listener(camera::CameraListener *listener) override { this->listeners_.push_back(listener); }
  camera::CameraImageReader *create_image_reader() override { return new JpegImageReader(); }
  void request_image(camera::CameraRequester requester) override;
  void start_stream(camera::CameraRequester requester) override;
  void stop_stream(camera::CameraRequester requester) override;

  void set_i2c_bus(i2c::I2CBus *bus) { this->i2c_bus_ = bus; }
  void set_reset_pin(int pin) { this->reset_pin_ = pin; }
  void set_pwdn_pin(int pin) { this->pwdn_pin_ = pin; }
  void set_init_ldo(bool init_ldo) { this->init_ldo_ = init_ldo; }
  void set_jpeg_quality(int quality) { this->jpeg_quality_ = quality; }
  void set_sensor_mode(SensorMode mode) { this->sensor_mode_ = mode; }
  void set_max_update_interval(uint32_t ms) { this->max_update_interval_ = ms; }
  void set_settle_frames(int frames) { this->settle_frames_ = frames; }
  void set_mirror(bool horizontal, bool vertical) {
    this->hmirror_ = horizontal;
    this->vflip_ = vertical;
  }

  /// Identifies the sensor, brings up esp_video, and logs which V4L2 devices
  /// registered. Diagnostic; nothing else needs it.
  void probe();

  /// Writes the sensor's test-pattern register: 0 is the live image, 1..4 one
  /// of its colour bars. Proves the CSI link, the ISP and the encoder without
  /// depending on optics or exposure. Takes effect on the next frame.
  void set_test_pattern(int pattern);

  /// Registers a consumer of raw ISP frames. Up to 8; call during setup.
  void add_raw_sink(RawVideoSink *sink);
  /// While any registered sink has streaming on, the capture session stays
  /// open and every frame the sensor produces is handed to all sinks on the
  /// capture task. Safe to call from any task.
  void set_raw_streaming(RawVideoSink *sink, bool on);

 protected:
  /// A finished frame, in flight from the capture task to the main loop.
  struct Frame {
    uint8_t *jpeg;
    size_t len;
    uint16_t width;
    uint16_t height;
    uint8_t requesters;
    uint32_t taken_ms;
  };

  bool has_requested_image_() const { return this->single_requesters_ != 0 || this->stream_requesters_ != 0; }
  /// Empties the frame queue, freeing what was in it.
  void drain_queue_();

  /// One-shot esp_video_init() plus sensor-mode selection. Returns false and logs on failure.
  bool ensure_video_init_();
  /// Reads the sensor's chip id over plain I2C, before any CSI is involved.
  void identify_();

  static void capture_task_(void *arg);
  void capture_loop_();
  bool open_session_();
  void close_session_();
  bool open_encoder_();
  void close_encoder_();
  /// Pulls one raw frame, hands it to the raw sinks, and - if `want_jpeg` -
  /// encodes it into `out`. Returns -1 if the session is unhealthy, 0 if a
  /// frame went by without a JPEG being wanted, 1 if `out` holds a JPEG.
  int grab_frame_(Frame *out, bool want_jpeg);
  bool encode_(const uint8_t *raw, size_t raw_len, Frame *out);

  i2c::I2CBus *i2c_bus_{nullptr};
  int reset_pin_{-1};
  int pwdn_pin_{-1};
  bool init_ldo_{false};
  int jpeg_quality_{30};
  SensorMode sensor_mode_{SENSOR_MODE_1920X1080};
  uint32_t max_update_interval_{333};
  int settle_frames_{24};
  bool hmirror_{false};
  bool vflip_{false};

  bool video_ready_{false};
  TaskHandle_t task_{nullptr};
  QueueHandle_t frame_queue_{nullptr};

  // Main-loop side.
  std::atomic<uint8_t> single_requesters_{0};
  std::atomic<uint8_t> stream_requesters_{0};
  /// Set by the main loop when it can take a frame; the capture task grabs one
  /// frame per set, so nothing is ever produced that the loop did not ask for.
  std::atomic<bool> frame_wanted_{false};
  /// One bit per registered raw sink that currently wants frames.
  std::atomic<uint8_t> raw_active_{0};
  std::vector<RawVideoSink *> raw_sinks_;
  uint32_t raw_sequence_{0};
  std::shared_ptr<JpegImage> current_image_;
  std::vector<camera::CameraListener *> listeners_;
  uint32_t last_update_{0};
  uint32_t published_at_{0};
  uint32_t frames_published_{0};

  // Capture-task side: the open V4L2 session, if any.
  static constexpr uint32_t CAPTURE_BUFFERS = 2;
  int cap_fd_{-1};
  uint8_t *cap_mem_[CAPTURE_BUFFERS] = {};
  size_t cap_len_[CAPTURE_BUFFERS] = {};
  bool cap_streaming_{false};
  uint32_t width_{0};
  uint32_t height_{0};
  uint32_t pixfmt_{0};
  int enc_fd_{-1};
  uint8_t *enc_out_{nullptr};
  size_t enc_out_len_{0};
  uint8_t *enc_cap_{nullptr};
  size_t enc_cap_len_{0};
  bool enc_streaming_{false};
  /// Set when the capture format is not one the JPEG encoder takes, so frames
  /// go through the PPA into the encoder's input buffer as RGB565.
  bool convert_for_jpeg_{false};
  ppa_client_handle_t ppa_{nullptr};
  uint32_t session_frames_{0};
  uint32_t session_errors_{0};
  uint32_t last_request_ms_{0};
};

}  // namespace p4_csi_camera
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
