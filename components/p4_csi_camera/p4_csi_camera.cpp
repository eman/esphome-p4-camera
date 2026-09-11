#include "p4_csi_camera.h"

#ifdef USE_ESP32_VARIANT_ESP32P4

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <driver/i2c_master.h>
#include <linux/videodev2.h>

#include "esp_cam_sensor_types.h"
#include "esp_heap_caps.h"
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "esp_video_isp_pipeline.h"
#include "hal/isp_ll.h"
#include "ov02c10.h"

#include "esphome/core/application.h"
#include "esp_timer.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace p4_csi_camera {

static const char *const TAG = "p4_csi_camera";

// The panel's sensor, read from the device: 0x300A..0x300B hold OmniVision's
// product id, and this one is 0x5602.
static const uint8_t SENSOR_ADDR = 0x36;
static const uint16_t OV02C10_PID = 0x5602;
// How long a session outlives the last request. Long enough that Home
// Assistant's stream keep-alives (every 5 s from the API side) never see a
// cold start; short enough that the sensor is not left streaming at 800 Mbps
// for a still nobody asked for.
static const uint32_t IDLE_CLOSE_MS = 6000;
// Bound on waiting for the pipeline. A dequeue that never returns is what
// tripped the watchdog the first time, and "no frame arrived" is a result.
static const uint32_t FRAME_WAIT_MS = 2000;
// A frame older than this when the loop gets to it is dropped, not published.
static const uint32_t FRAME_MAX_AGE_MS = 1500;
// How long one slow API client may hold the current image before the camera
// carries on without it.
static const uint32_t IMAGE_HOLD_LIMIT_MS = 4000;

static const char *fourcc(uint32_t f, char *buf) {
  buf[0] = (char) (f & 0xFF);
  buf[1] = (char) ((f >> 8) & 0xFF);
  buf[2] = (char) ((f >> 16) & 0xFF);
  buf[3] = (char) ((f >> 24) & 0xFF);
  buf[4] = 0;
  return buf;
}

/* ---------------- JpegImage / JpegImageReader ---------------- */

JpegImage::~JpegImage() {
  if (this->data_ != nullptr)
    heap_caps_free(this->data_);
}

void JpegImageReader::set_image(std::shared_ptr<camera::CameraImage> image) {
  this->image_ = std::move(image);
  this->offset_ = 0;
}
size_t JpegImageReader::available() const {
  if (!this->image_)
    return 0;
  return this->image_->get_data_length() - this->offset_;
}
uint8_t *JpegImageReader::peek_data_buffer() { return this->image_->get_data_buffer() + this->offset_; }
void JpegImageReader::consume_data(size_t consumed) { this->offset_ += consumed; }
void JpegImageReader::return_image() { this->image_.reset(); }

/* ---------------- main-loop side ---------------- */

void P4CsiCamera::setup() {
  // No esp_log_level_set() here: ESPHome builds IDF with
  // CONFIG_LOG_DYNAMIC_LEVEL_CONTROL=n, which makes it a no-op. What the IDF
  // layers log is decided at compile time (CONFIG_LOG_MAXIMUM_LEVEL_INFO, see
  // __init__.py) and at ESPHome's logger, where the `esp-idf` tag is held to
  // INFO (packages/core.yaml).

  this->frame_queue_ = xQueueCreate(1, sizeof(Frame));
  if (this->frame_queue_ == nullptr) {
    ESP_LOGE(TAG, "could not create the frame queue");
    this->mark_failed();
    return;
  }
  // Its own stack, on the other core: the main task - and with it LVGL - is
  // pinned to core 0 (CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0), and a 4 MB frame
  // copy plus a hardware encode should not steal cycles from the display.
  if (xTaskCreatePinnedToCore(P4CsiCamera::capture_task_, "cam_capture", 8192, this, 3, &this->task_, 1) !=
      pdPASS) {
    ESP_LOGE(TAG, "could not start the capture task");
    this->task_ = nullptr;
    this->mark_failed();
  }
}

void P4CsiCamera::loop() {
  if (!this->current_image_ && !this->has_requested_image_())
    return;

  const uint32_t now = App.get_loop_component_start_time();

  // Every reader has returned the image: let its buffer go. A reader that
  // cannot finish sending - a client that has stopped reading its socket -
  // must not hold the camera for everyone else, so after a while the image
  // is let go of here too; the reader keeps its own reference until it is done.
  if (this->current_image_) {
    if (this->current_image_.use_count() == 1) {
      this->current_image_.reset();
    } else if (now - this->published_at_ > IMAGE_HOLD_LIMIT_MS) {
      ESP_LOGD(TAG, "an API client has held image %u for %u ms; moving on without it",
               (unsigned) this->frames_published_, (unsigned) (now - this->published_at_));
      this->current_image_.reset();
    }
  }

  // A frame the task produced on request: publish it, unless it sat in the
  // queue long enough to be a picture of the past.
  Frame frame{};
  if (xQueueReceive(this->frame_queue_, &frame, 0) == pdTRUE) {
    if (millis() - frame.taken_ms > FRAME_MAX_AGE_MS || this->current_image_) {
      heap_caps_free(frame.jpeg);
    } else {
      this->current_image_ =
          std::make_shared<JpegImage>(frame.jpeg, frame.len, frame.width, frame.height, frame.requesters);
      this->frames_published_++;
      this->published_at_ = now;
      ESP_LOGD(TAG, "image %u: %ux%u, %u bytes", (unsigned) this->frames_published_, frame.width, frame.height,
               (unsigned) frame.len);
      for (auto *listener : this->listeners_)
        listener->on_camera_image(this->current_image_);
      this->last_update_ = now;
      this->single_requesters_ = 0;
      return;
    }
  }

  // Ask for the next one only when it could be published straight away.
  if (!this->has_requested_image_() || this->current_image_ || this->frame_wanted_)
    return;
  if (now - this->last_update_ < this->max_update_interval_)
    return;
  this->frame_wanted_ = true;
  if (this->task_ != nullptr)
    xTaskNotifyGive(this->task_);
}

void P4CsiCamera::request_image(camera::CameraRequester requester) {
  this->single_requesters_ |= (1u << requester);
  if (this->task_ != nullptr)
    xTaskNotifyGive(this->task_);
}

void P4CsiCamera::start_stream(camera::CameraRequester requester) {
  for (auto *listener : this->listeners_)
    listener->on_stream_start();
  this->stream_requesters_ |= (1u << requester);
  if (this->task_ != nullptr)
    xTaskNotifyGive(this->task_);
}

void P4CsiCamera::stop_stream(camera::CameraRequester requester) {
  for (auto *listener : this->listeners_)
    listener->on_stream_stop();
  this->stream_requesters_ &= (uint8_t) ~(1u << requester);
}

void P4CsiCamera::add_raw_sink(RawVideoSink *sink) {
  if (this->raw_sinks_.size() >= 8) {
    ESP_LOGE(TAG, "no room for another raw video sink");
    return;
  }
  this->raw_sinks_.push_back(sink);
}

void P4CsiCamera::set_raw_streaming(RawVideoSink *sink, bool on) {
  for (size_t i = 0; i < this->raw_sinks_.size(); i++) {
    if (this->raw_sinks_[i] != sink)
      continue;
    if (on)
      this->raw_active_ |= (uint8_t) (1u << i);
    else
      this->raw_active_ &= (uint8_t) ~(1u << i);
    if (on && this->task_ != nullptr)
      xTaskNotifyGive(this->task_);
    return;
  }
  ESP_LOGW(TAG, "set_raw_streaming: unknown sink");
}

void P4CsiCamera::dump_config() {
  ESP_LOGCONFIG(TAG, "MIPI-CSI camera '%s':", this->get_name().c_str());
  ESP_LOGCONFIG(TAG, "  Sensor: OV02C10 at 0x%02X on the shared I2C bus", SENSOR_ADDR);
  ESP_LOGCONFIG(TAG, "  Mode: %s", this->sensor_mode_ == SENSOR_MODE_1288X728 ? "1288x728, one lane"
                                                                              : "1920x1080, two lanes");
  ESP_LOGCONFIG(TAG, "  JPEG quality: %d", this->jpeg_quality_);
  ESP_LOGCONFIG(TAG, "  Max frame interval: %u ms, settle frames: %d", (unsigned) this->max_update_interval_,
                this->settle_frames_);
  ESP_LOGCONFIG(TAG, "  Mirror: %s, flip: %s", YESNO(this->hmirror_), YESNO(this->vflip_));
  ESP_LOGCONFIG(TAG, "  Reset pin: %d, power-down pin: %d", this->reset_pin_, this->pwdn_pin_);
  ESP_LOGCONFIG(TAG, "  LDO: %s", this->init_ldo_ ? "brought up here" : "already up for the DSI PHY");
}

/* ---------------- bring-up ---------------- */

bool P4CsiCamera::ensure_video_init_() {
  if (this->video_ready_)
    return true;

  // ESPHome owns this bus (the GT911 is on it), and it uses the i2c_master
  // driver - so rather than letting esp_video open a second master on the same
  // pins, borrow the handle. get_port() is the only public route to it;
  // InternalI2CBus is what every ESP32 bus actually is.
  auto *internal = static_cast<i2c::InternalI2CBus *>(this->i2c_bus_);
  i2c_master_bus_handle_t bus = nullptr;
  const esp_err_t err = i2c_master_get_bus_handle((i2c_port_num_t) internal->get_port(), &bus);
  if (err != ESP_OK || bus == nullptr) {
    ESP_LOGE(TAG, "could not borrow the I2C bus handle for port %d: %s", internal->get_port(),
             esp_err_to_name(err));
    return false;
  }

  // Assigned field by field rather than with designated initialisers: the SCCB
  // config carries an anonymous union, and C++ will not let some members be
  // designated and not others.
  esp_video_init_sccb_config_t sccb = {};
  sccb.init_sccb = false;
  sccb.i2c_handle = bus;
  sccb.freq = 100000;

  esp_video_init_csi_config_t csi = {};
  csi.sccb_config = sccb;
  csi.reset_pin = (gpio_num_t) this->reset_pin_;
  csi.pwdn_pin = (gpio_num_t) this->pwdn_pin_;
  // The DSI PHY already claimed this rail at boot (esp_ldo channel 3, 2.5V).
  // Letting esp_video claim it again fails, or worse, succeeds and fights.
  csi.dont_init_ldo = !this->init_ldo_;

  esp_video_init_config_t config = {};
  config.csi = &csi;

  const esp_err_t init = esp_video_init(&config);
  if (init != ESP_OK) {
    ESP_LOGE(TAG, "esp_video_init failed: %s", esp_err_to_name(init));
    return false;
  }
  this->video_ready_ = true;

  // The driver comes up in its default mode, 1920x1080 over two lanes. Any
  // other mode is a full register reload, done once here while nothing is
  // streaming; esp_video re-derives the capture geometry from it.
  if (this->sensor_mode_ != SENSOR_MODE_1920X1080) {
    const esp_cam_sensor_format_t *fmt = ov02c10_get_format_by_index(this->sensor_mode_);
    const int fd = open("/dev/video0", O_RDWR);
    if (fmt == nullptr || fd < 0) {
      ESP_LOGW(TAG, "could not select sensor mode %d; staying at the default", (int) this->sensor_mode_);
    } else {
      // The driver keeps the pointer it is given as its current format, so
      // this has to be its own table entry - not a copy on this stack, which
      // it would go on dereferencing after this function returns.
      if (ioctl(fd, VIDIOC_S_SENSOR_FMT, const_cast<esp_cam_sensor_format_t *>(fmt)) != 0) {
        ESP_LOGW(TAG, "VIDIOC_S_SENSOR_FMT failed (errno %d); staying at the default", errno);
      } else {
        ESP_LOGI(TAG, "sensor mode: %s", fmt->name);
      }
    }
    if (fd >= 0)
      close(fd);
  }
  return true;
}

void P4CsiCamera::identify_() {
  // 16-bit register addressing: write the register, then read one byte.
  uint8_t id[3] = {};
  for (int i = 0; i < 3; i++) {
    const uint8_t reg[2] = {0x30, (uint8_t) (0x0A + i)};
    if (this->i2c_bus_->write_readv(SENSOR_ADDR, reg, 2, &id[i], 1) != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "nothing answered at 0x%02X", SENSOR_ADDR);
      return;
    }
  }

  const uint16_t pid = ((uint16_t) id[0] << 8) | id[1];
  ESP_LOGI(TAG, "sensor at 0x%02X: chip id 0x%02X%02X%02X%s", SENSOR_ADDR, id[0], id[1], id[2],
           pid == OV02C10_PID ? "  (OV02C10)" : "  (unrecognised)");
}

void P4CsiCamera::probe() {
  this->identify_();

  if (!this->ensure_video_init_())
    return;

  // esp_video numbers its devices by role: capture low, JPEG around 10, ISP
  // around 20. Which ones registered says how far the pipeline got.
  bool have_capture = false;
  for (int i = 0; i < 26; i++) {
    char path[16];
    snprintf(path, sizeof(path), "/dev/video%d", i);
    const int fd = open(path, O_RDWR);
    if (fd < 0)
      continue;
    struct v4l2_capability cap = {};
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
      ESP_LOGI(TAG, "%s: '%s' / '%s'", path, (const char *) cap.driver, (const char *) cap.card);
      if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)
        have_capture = true;
    }
    close(fd);
  }

  if (!have_capture) {
    ESP_LOGW(TAG, "no capture device: the ISP and JPEG stages are up, but nothing claimed the "
                  "sensor. Either it did not answer on SCCB, or the idf/ov02c10 component was "
                  "not linked in - see its CMakeLists.");
  }
}

void P4CsiCamera::set_test_pattern(int pattern) {
  // Straight to the sensor register, on the bus this component already owns:
  // 0x4503 bits [1:0] pick the bar, bit 7 enables. The sensor applies it on
  // the next frame, streaming or not.
  uint8_t value = 0;
  const uint8_t reg[2] = {0x45, 0x03};
  if (this->i2c_bus_->write_readv(SENSOR_ADDR, reg, 2, &value, 1) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "could not read the test-pattern register");
    return;
  }

  if (pattern <= 0) {
    value &= (uint8_t) ~0x80;
  } else {
    value = (uint8_t) ((value & ~0x03) | ((pattern - 1) & 0x03));
    value |= 0x80;
  }

  const uint8_t write[3] = {0x45, 0x03, value};
  if (this->i2c_bus_->write_readv(SENSOR_ADDR, write, 3, nullptr, 0) != i2c::ERROR_OK) {
    ESP_LOGW(TAG, "could not write the test-pattern register");
    return;
  }
  ESP_LOGI(TAG, "test pattern %d (reg 0x4503 = 0x%02X)", pattern, value);
}

/* ---------------- capture task ---------------- */

void P4CsiCamera::capture_task_(void *arg) { static_cast<P4CsiCamera *>(arg)->capture_loop_(); }

void P4CsiCamera::capture_loop_() {
  for (;;) {
    const bool open = this->cap_fd_ >= 0;
    const bool raw = this->raw_active_ != 0;
    // Raw consumers take every frame, so with one active the loop just runs.
    // Otherwise: idle, sleep until the main loop wants a frame; session open,
    // wake often enough to notice it has gone quiet and close it.
    if (!raw)
      ulTaskNotifyTake(pdTRUE, open ? pdMS_TO_TICKS(100) : portMAX_DELAY);

    if (this->sweep_requested_.exchange(false)) {
      if (open || this->open_session_())
        this->run_gain_sweep_();
      continue;
    }

    const bool want_jpeg = this->frame_wanted_;
    if (!want_jpeg && !raw) {
      if (open && millis() - this->last_request_ms_ > IDLE_CLOSE_MS) {
        this->close_session_();
        this->drain_queue_();
      }
      continue;
    }
    this->last_request_ms_ = millis();

    if (!open && !this->open_session_()) {
      // Do not spin on a pipeline that will not come up: drop the still
      // requests, and give a stream a second before it tries again.
      this->single_requesters_ = 0;
      this->frame_wanted_ = false;
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    Frame frame{};
    const int got = this->grab_frame_(&frame, want_jpeg);
    if (got < 0) {
      if (++this->session_errors_ >= 3) {
        ESP_LOGW(TAG, "closing the capture session after %u consecutive failures",
                 (unsigned) this->session_errors_);
        this->close_session_();
        this->single_requesters_ = 0;
        this->frame_wanted_ = false;
      }
      continue;
    }
    this->session_errors_ = 0;
    if (got == 0)
      continue;
    frame.taken_ms = millis();

    // One JPEG per request from the loop, so the queue never holds a stale
    // picture waiting for the next request to publish it.
    this->frame_wanted_ = false;
    if (xQueueSend(this->frame_queue_, &frame, 0) != pdTRUE)
      heap_caps_free(frame.jpeg);
  }
}

/// Dequeues `frames` frames and returns the mean luma of the last one.
uint32_t P4CsiCamera::measure_luma_(int frames) {
  uint32_t luma = 0;
  for (int f = 0; f < frames; f++) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    int rc = -1;
    for (uint32_t waited = 0; waited < FRAME_WAIT_MS && rc != 0; waited += 10) {
      rc = ioctl(this->cap_fd_, VIDIOC_DQBUF, &buf);
      if (rc != 0)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (rc != 0)
      return 0;
    const uint8_t *data = this->cap_mem_[buf.index];
    uint32_t sum = 0, n = 0;
    for (size_t i = 1; i < buf.bytesused; i += 3 * 97) {
      sum += data[i];
      n++;
    }
    luma = n ? sum / n : 0;
    ioctl(this->cap_fd_, VIDIOC_QBUF, &buf);
  }
  return luma;
}

void P4CsiCamera::run_gain_sweep_() {
  // Auto exposure paused; registers written straight to the sensor, so what
  // is measured is the sensor, not the driver's idea of it.
  esp_video_isp_pipeline_set_agc_status(ESP_VIDEO_ISP_PIPELINE_AGC_DISABLE);
  vTaskDelay(pdMS_TO_TICKS(100));
  auto w = [&](uint16_t reg, uint8_t val) {
    const uint8_t wr[3] = {(uint8_t) (reg >> 8), (uint8_t) reg, val};
    this->i2c_bus_->write_readv(SENSOR_ADDR, wr, 3, nullptr, 0);
  };
  auto exposure = [&](uint32_t lines) {
    w(0x3500, (uint8_t) (lines >> 16));
    w(0x3501, (uint8_t) (lines >> 8));
    w(0x3502, (uint8_t) lines);
  };
  auto gain = [&](uint16_t r3508, uint16_t r350a) {
    w(0x3508, (uint8_t) (r3508 >> 8));
    w(0x3509, (uint8_t) r3508);
    w(0x350a, (uint8_t) (r350a >> 8));
    w(0x350b, (uint8_t) r350a);
  };
  ESP_LOGI(TAG, "sweep: exposure in lines, gain 1x");
  gain(0x0100, 0x0100);
  for (uint32_t e : {64u, 1149u, 2313u, 4600u})
    exposure(e), ESP_LOGI(TAG, "sweep:   exposure %5u lines -> luma %u", (unsigned) e, (unsigned) this->measure_luma_(6));
  ESP_LOGI(TAG, "sweep: 0x350a:0x350b at 2313 lines");
  exposure(2313);
  for (uint16_t g : {0x0100, 0x0400, 0x0800, 0x0FF0})
    gain(0x0100, g), ESP_LOGI(TAG, "sweep:   0x%04X -> luma %u", g, (unsigned) this->measure_luma_(6));
  ESP_LOGI(TAG, "sweep: 0x3508:0x3509 at 2313 lines");
  for (uint16_t g : {0x0100, 0x0800, 0x0F80})
    gain(g, 0x0100), ESP_LOGI(TAG, "sweep:   0x%04X -> luma %u", g, (unsigned) this->measure_luma_(6));
  gain(0x0100, 0x0100);
  exposure(1149);
  esp_video_isp_pipeline_set_agc_status(ESP_VIDEO_ISP_PIPELINE_AGC_ENABLE);
  this->last_request_ms_ = millis();
}

void P4CsiCamera::drain_queue_() {
  Frame frame{};
  while (xQueueReceive(this->frame_queue_, &frame, 0) == pdTRUE)
    heap_caps_free(frame.jpeg);
}

bool P4CsiCamera::open_session_() {
  if (!this->ensure_video_init_())
    return false;

  this->cap_fd_ = open("/dev/video0", O_RDWR | O_NONBLOCK);
  if (this->cap_fd_ < 0) {
    ESP_LOGE(TAG, "/dev/video0 did not open (errno %d)", errno);
    return false;
  }
  const int fd = this->cap_fd_;
  char cc[5];

  // YUV 4:2:0 out of the ISP: the hardware JPEG encoder and the hardware
  // H.264 encoder both take it, and it is a quarter smaller than RGB565.
  // RGB565 is the fallback, then whatever comes first.
  uint32_t chosen = 0;
  for (uint32_t i = 0;; i++) {
    struct v4l2_fmtdesc desc = {};
    desc.index = i;
    desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_ENUM_FMT, &desc) != 0)
      break;
    if (chosen == 0 || desc.pixelformat == V4L2_PIX_FMT_YUV420 ||
        (desc.pixelformat == V4L2_PIX_FMT_RGB565 && chosen != V4L2_PIX_FMT_YUV420))
      chosen = desc.pixelformat;
  }
  if (chosen == 0) {
    ESP_LOGE(TAG, "the capture device offers no formats");
    this->close_session_();
    return false;
  }

  // The geometry is the sensor's; only the pixel format is ours to choose.
  // Setting the format is also what sizes the buffers: reading it back without
  // setting it first leaves sizeimage 0 and STREAMON fails with EINVAL.
  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
    ESP_LOGE(TAG, "VIDIOC_G_FMT failed (errno %d)", errno);
    this->close_session_();
    return false;
  }
  fmt.fmt.pix.pixelformat = chosen;
  if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
    ESP_LOGE(TAG, "VIDIOC_S_FMT failed (errno %d)", errno);
    this->close_session_();
    return false;
  }
  this->width_ = fmt.fmt.pix.width;
  this->height_ = fmt.fmt.pix.height;
  this->pixfmt_ = fmt.fmt.pix.pixelformat;

  struct v4l2_requestbuffers req = {};
  req.count = CAPTURE_BUFFERS;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
    ESP_LOGE(TAG, "REQBUFS failed (errno %d)", errno);
    this->close_session_();
    return false;
  }
  for (uint32_t i = 0; i < req.count && i < CAPTURE_BUFFERS; i++) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) != 0) {
      ESP_LOGE(TAG, "QUERYBUF %u failed (errno %d)", (unsigned) i, errno);
      this->close_session_();
      return false;
    }
    void *mem = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    if (mem == MAP_FAILED) {
      ESP_LOGE(TAG, "mmap %u failed", (unsigned) i);
      this->close_session_();
      return false;
    }
    this->cap_mem_[i] = (uint8_t *) mem;
    this->cap_len_[i] = buf.length;
    if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGE(TAG, "QBUF %u failed (errno %d)", (unsigned) i, errno);
      this->close_session_();
      return false;
    }
  }

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
    ESP_LOGE(TAG, "STREAMON failed (errno %d)", errno);
    this->close_session_();
    return false;
  }
  this->cap_streaming_ = true;

  // Stop the ISP's error interrupts, now that STREAMON has created the ISP
  // processor (which enables them). Every one of them prints a line from
  // inside the interrupt handler - ESP_EARLY_LOGE over a 115200-baud UART,
  // several milliseconds with interrupts off - and a frame that trips them
  // continuously (a FIFO overflow, a geometry mismatch) turns into an
  // interrupt storm that the interrupt watchdog ends 800 ms later. That was
  // the reboot behind roughly one capture in ten. esp_video does exactly this
  // itself, but only on ESP-IDF 5.5.6 and later; this build has 5.5.5. The
  // raw status still records them, and close_session_() reports it.
  isp_ll_enable_intr(&ISP, ISP_LL_EVENT_ERROR_MASK, false);

  // Orientation, through the sensor's own mirror and flip bits. esp_video
  // maps these controls onto the driver's HMIRROR and VFLIP parameters.
  {
    struct v4l2_ext_control ctl[2] = {};
    ctl[0].id = V4L2_CID_HFLIP;
    ctl[0].value = this->hmirror_ ? 1 : 0;
    ctl[1].id = V4L2_CID_VFLIP;
    ctl[1].value = this->vflip_ ? 1 : 0;
    struct v4l2_ext_controls ctls = {};
    ctls.ctrl_class = V4L2_CTRL_CLASS_USER;
    ctls.count = 2;
    ctls.controls = ctl;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctls) != 0)
      ESP_LOGW(TAG, "could not set mirror/flip (errno %d)", errno);
  }

  this->session_frames_ = 0;
  this->session_errors_ = 0;

  if (!this->open_encoder_()) {
    this->close_session_();
    return false;
  }

  ESP_LOGI(TAG, "capture session open: %ux%u '%s', %u buffers of %u KB, JPEG quality %d", (unsigned) this->width_,
           (unsigned) this->height_, fourcc(this->pixfmt_, cc), (unsigned) req.count,
           (unsigned) (this->cap_len_[0] / 1024), this->jpeg_quality_);
  return true;
}

void P4CsiCamera::close_session_() {
  this->close_encoder_();

  if (this->cap_fd_ >= 0) {
    if (this->cap_streaming_) {
      // Where auto exposure ended up, for judging the tuning file against
      // saved frames: exposure in lines of the sensor, gain as an index into
      // the driver's gain table.
      struct v4l2_ext_control ctl[2] = {};
      ctl[0].id = V4L2_CID_EXPOSURE;
      ctl[1].id = V4L2_CID_GAIN;
      struct v4l2_ext_controls ctls = {};
      ctls.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
      ctls.count = 1;
      ctls.controls = &ctl[0];
      const bool have_exp = ioctl(this->cap_fd_, VIDIOC_G_EXT_CTRLS, &ctls) == 0;
      ctls.ctrl_class = V4L2_CTRL_CLASS_USER;
      ctls.controls = &ctl[1];
      const bool have_gain = ioctl(this->cap_fd_, VIDIOC_G_EXT_CTRLS, &ctls) == 0;
      ESP_LOGI(TAG, "auto exposure ended at exposure %s%ld, gain index %s%ld", have_exp ? "" : "?",
               (long) ctl[0].value, have_gain ? "" : "?", (long) ctl[1].value);
      // And what the sensor itself holds, read back over I2C: exposure at
      // 0x3500..0x3502, analogue gain at 0x3508:0x3509, digital gain at
      // 0x350a..0x350c. If these do not follow the values above, the gain
      // writes are not landing.
      {
        uint8_t regs[13] = {};
        bool ok = true;
        for (int i = 0; i < 13 && ok; i++) {
          const uint8_t reg[2] = {0x35, (uint8_t) i};
          ok = this->i2c_bus_->write_readv(SENSOR_ADDR, reg, 2, &regs[i], 1) == i2c::ERROR_OK;
        }
        if (ok)
          ESP_LOGI(TAG, "sensor registers: exposure 0x%02X%02X%02X, analogue gain 0x%02X%02X, digital gain 0x%02X%02X%02X",
                   regs[0], regs[1], regs[2], regs[8], regs[9], regs[10], regs[11], regs[12]);
        else
          ESP_LOGW(TAG, "could not read the sensor's exposure and gain registers back");
      }

      // Read before STREAMOFF: it deletes the ISP processor, and the raw
      // status is only meaningful while the block is clocked.
      const uint32_t raw = isp_ll_get_intr_raw(&ISP) & ISP_LL_EVENT_ERROR_MASK;
      if (raw != 0) {
        ESP_LOGW(TAG, "the ISP flagged errors during this session (raw 0x%08x):%s%s%s%s", (unsigned) raw,
                 (raw & ISP_LL_EVENT_HVNUM_SETTING_ERR) ? " frame-geometry-mismatch" : "",
                 (raw & ISP_LL_EVENT_ASYNC_FIFO_OVF) ? " fifo-overflow" : "",
                 (raw & (ISP_LL_EVENT_DATA_TYPE_ERR | ISP_LL_EVENT_DATA_TYPE_SETTING_ERR)) ? " data-type" : "",
                 (raw & ISP_LL_EVENT_CROP_ERR) ? " crop" : "");
      }
      isp_ll_clear_intr(&ISP, ISP_LL_EVENT_ERROR_MASK);

      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      ioctl(this->cap_fd_, VIDIOC_STREAMOFF, &type);
      this->cap_streaming_ = false;
    }
    // Order matters: stop the stream first, then unmap, then close - the
    // failure paths used to close the descriptor with DMA still armed into
    // buffers nothing owned any more.
    for (uint32_t i = 0; i < CAPTURE_BUFFERS; i++) {
      if (this->cap_mem_[i] != nullptr)
        munmap(this->cap_mem_[i], this->cap_len_[i]);
      this->cap_mem_[i] = nullptr;
      this->cap_len_[i] = 0;
    }
    close(this->cap_fd_);
    this->cap_fd_ = -1;
    ESP_LOGI(TAG, "capture session closed after %u frames", (unsigned) this->session_frames_);
  }
}

bool P4CsiCamera::open_encoder_() {
  // /dev/video10 is a memory-to-memory device: the raw frame goes in on the
  // OUTPUT queue and the JPEG comes back on the CAPTURE queue. Blocking, so a
  // dequeue waits for the encoder rather than polling it.
  this->enc_fd_ = open("/dev/video10", O_RDWR);
  if (this->enc_fd_ < 0) {
    ESP_LOGE(TAG, "/dev/video10 did not open (errno %d)", errno);
    return false;
  }
  const int fd = this->enc_fd_;

  // The JPEG encoder on pre-revision-3 ESP32-P4 silicon (this panel's) takes
  // RGB565 and YUV422 but not the YUV 4:2:0 the H.264 encoder needs, so when
  // the capture runs in YUV 4:2:0 the pixel processing accelerator converts
  // each still into the encoder's input buffer as RGB565. Hardware, and no
  // slower than the memcpy it replaces.
  this->convert_for_jpeg_ = this->pixfmt_ == V4L2_PIX_FMT_YUV420;
  if (this->convert_for_jpeg_ && this->ppa_ == nullptr) {
    ppa_client_config_t ppa_cfg = {};
    ppa_cfg.oper_type = PPA_OPERATION_SRM;
    ppa_cfg.max_pending_trans_num = 1;
    if (ppa_register_client(&ppa_cfg, &this->ppa_) != ESP_OK) {
      ESP_LOGE(TAG, "could not register a PPA client for YUV to RGB conversion");
      this->ppa_ = nullptr;
      return false;
    }
  }

  struct v4l2_format out_fmt = {};
  out_fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  out_fmt.fmt.pix.width = this->width_;
  out_fmt.fmt.pix.height = this->height_;
  out_fmt.fmt.pix.pixelformat = this->convert_for_jpeg_ ? V4L2_PIX_FMT_RGB565 : this->pixfmt_;
  if (ioctl(fd, VIDIOC_S_FMT, &out_fmt) != 0) {
    ESP_LOGE(TAG, "JPEG: S_FMT on the output queue failed (errno %d)", errno);
    return false;
  }
  struct v4l2_format cap_fmt = {};
  cap_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  cap_fmt.fmt.pix.width = this->width_;
  cap_fmt.fmt.pix.height = this->height_;
  cap_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG;
  if (ioctl(fd, VIDIOC_S_FMT, &cap_fmt) != 0) {
    ESP_LOGE(TAG, "JPEG: S_FMT on the capture queue failed (errno %d)", errno);
    return false;
  }

  // Extended controls, not VIDIOC_S_CTRL: esp_video implements this only
  // through the S_EXT_CTRLS path with the JPEG control class, and the simple
  // call returns EINVAL. The encoder's own default is near-lossless: 800 KB
  // for a 1080p frame.
  struct v4l2_ext_control control = {};
  control.id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
  control.value = this->jpeg_quality_;
  struct v4l2_ext_controls controls = {};
  controls.ctrl_class = V4L2_CID_JPEG_CLASS;
  controls.count = 1;
  controls.controls = &control;
  if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
    ESP_LOGW(TAG, "could not set JPEG quality to %d (errno %d); using the encoder default", this->jpeg_quality_,
             errno);
  }

  struct v4l2_requestbuffers out_req = {};
  out_req.count = 1;
  out_req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  out_req.memory = V4L2_MEMORY_MMAP;
  struct v4l2_requestbuffers cap_req = {};
  cap_req.count = 1;
  cap_req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  cap_req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_REQBUFS, &out_req) != 0 || ioctl(fd, VIDIOC_REQBUFS, &cap_req) != 0) {
    ESP_LOGE(TAG, "JPEG: REQBUFS failed (errno %d)", errno);
    return false;
  }

  struct v4l2_buffer out_buf = {};
  out_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  out_buf.memory = V4L2_MEMORY_MMAP;
  struct v4l2_buffer cap_buf = {};
  cap_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  cap_buf.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_QUERYBUF, &out_buf) != 0 || ioctl(fd, VIDIOC_QUERYBUF, &cap_buf) != 0) {
    ESP_LOGE(TAG, "JPEG: QUERYBUF failed (errno %d)", errno);
    return false;
  }
  void *out_mem = mmap(nullptr, out_buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, out_buf.m.offset);
  if (out_mem == MAP_FAILED) {
    ESP_LOGE(TAG, "JPEG: mmap of the input buffer failed");
    return false;
  }
  this->enc_out_ = (uint8_t *) out_mem;
  this->enc_out_len_ = out_buf.length;
  void *cap_mem = mmap(nullptr, cap_buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, cap_buf.m.offset);
  if (cap_mem == MAP_FAILED) {
    ESP_LOGE(TAG, "JPEG: mmap of the output buffer failed");
    return false;
  }
  this->enc_cap_ = (uint8_t *) cap_mem;
  this->enc_cap_len_ = cap_buf.length;
  return true;
}

void P4CsiCamera::close_encoder_() {
  if (this->enc_fd_ < 0)
    return;
  if (this->enc_streaming_) {
    int type_out = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    int type_cap = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(this->enc_fd_, VIDIOC_STREAMOFF, &type_out);
    ioctl(this->enc_fd_, VIDIOC_STREAMOFF, &type_cap);
    this->enc_streaming_ = false;
  }
  if (this->enc_out_ != nullptr)
    munmap(this->enc_out_, this->enc_out_len_);
  if (this->enc_cap_ != nullptr)
    munmap(this->enc_cap_, this->enc_cap_len_);
  this->enc_out_ = nullptr;
  this->enc_cap_ = nullptr;
  close(this->enc_fd_);
  this->enc_fd_ = -1;
}

bool P4CsiCamera::encode_(const uint8_t *raw, size_t raw_len, Frame *out) {
  const int fd = this->enc_fd_;
  size_t copy;
  if (this->convert_for_jpeg_) {
    ppa_srm_oper_config_t srm = {};
    srm.in.buffer = raw;
    srm.in.pic_w = this->width_;
    srm.in.pic_h = this->height_;
    srm.in.block_w = this->width_;
    srm.in.block_h = this->height_;
    srm.in.srm_cm = PPA_SRM_COLOR_MODE_YUV420;
    // What the ISP was configured to emit (esp_video's defaults).
    srm.in.yuv_range = PPA_COLOR_RANGE_FULL;
    srm.in.yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601;
    srm.out.buffer = this->enc_out_;
    srm.out.buffer_size = this->enc_out_len_;
    srm.out.pic_w = this->width_;
    srm.out.pic_h = this->height_;
    srm.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    srm.rotation_angle = PPA_SRM_ROTATION_ANGLE_0;
    srm.scale_x = 1.0f;
    srm.scale_y = 1.0f;
    srm.mode = PPA_TRANS_MODE_BLOCKING;
    const esp_err_t err = ppa_do_scale_rotate_mirror(this->ppa_, &srm);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "PPA YUV420 to RGB565 conversion failed: %s", esp_err_to_name(err));
      return false;
    }
    copy = (size_t) this->width_ * this->height_ * 2;
  } else {
    copy = raw_len < this->enc_out_len_ ? raw_len : this->enc_out_len_;
    memcpy(this->enc_out_, raw, copy);
  }

  struct v4l2_buffer out_buf = {};
  out_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  out_buf.memory = V4L2_MEMORY_MMAP;
  out_buf.bytesused = copy;
  struct v4l2_buffer cap_buf = {};
  cap_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  cap_buf.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_QBUF, &out_buf) != 0 || ioctl(fd, VIDIOC_QBUF, &cap_buf) != 0) {
    ESP_LOGE(TAG, "JPEG: could not queue the buffers (errno %d)", errno);
    return false;
  }
  if (!this->enc_streaming_) {
    // Marked one at a time: if the second STREAMON fails, the first still has
    // to be stopped.
    int type_out = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    int type_cap = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type_out) != 0) {
      ESP_LOGE(TAG, "JPEG: STREAMON on the output queue failed (errno %d)", errno);
      return false;
    }
    this->enc_streaming_ = true;
    if (ioctl(fd, VIDIOC_STREAMON, &type_cap) != 0) {
      ESP_LOGE(TAG, "JPEG: STREAMON on the capture queue failed (errno %d)", errno);
      return false;
    }
  }

  struct v4l2_buffer done = {};
  done.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  done.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_DQBUF, &done) != 0) {
    ESP_LOGE(TAG, "JPEG: DQBUF failed (errno %d)", errno);
    return false;
  }
  // Take the input buffer back too, or the next QBUF finds it still queued.
  struct v4l2_buffer spent = {};
  spent.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  spent.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_DQBUF, &spent) != 0)
    ESP_LOGW(TAG, "JPEG: could not dequeue the input buffer (errno %d)", errno);

  const uint8_t *jpeg = this->enc_cap_;
  const size_t len = done.bytesused;
  const bool soi = len > 3 && jpeg[0] == 0xFF && jpeg[1] == 0xD8;
  const bool eoi = len > 3 && jpeg[len - 2] == 0xFF && jpeg[len - 1] == 0xD9;
  if (!soi || !eoi) {
    ESP_LOGE(TAG, "JPEG: %u bytes but SOI %s, EOI %s - discarded", (unsigned) len, YESNO(soi), YESNO(eoi));
    return false;
  }

  // A copy the API can hold for as long as it needs to send it, while the
  // encoder's buffer goes straight back to work.
  auto *data = (uint8_t *) heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (data == nullptr) {
    ESP_LOGE(TAG, "JPEG: no memory for a %u byte copy", (unsigned) len);
    return false;
  }
  memcpy(data, jpeg, len);
  out->jpeg = data;
  out->len = len;
  out->width = (uint16_t) this->width_;
  out->height = (uint16_t) this->height_;
  return true;
}

int P4CsiCamera::grab_frame_(Frame *out, bool want_jpeg) {
  const int fd = this->cap_fd_;
  for (;;) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    int rc = -1;
    for (uint32_t waited = 0; waited < FRAME_WAIT_MS; waited += 10) {
      rc = ioctl(fd, VIDIOC_DQBUF, &buf);
      if (rc == 0)
        break;
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        ESP_LOGE(TAG, "DQBUF failed (errno %d)", errno);
        return -1;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (rc != 0) {
      ESP_LOGE(TAG, "no frame in %u ms: the pipeline started but the sensor is not delivering",
               (unsigned) FRAME_WAIT_MS);
      return -1;
    }
    this->session_frames_++;

    // The first frames after STREAMON are at whatever exposure the sensor
    // woke up with; give auto exposure a few to converge.
    if (this->session_frames_ <= (uint32_t) this->settle_frames_) {
      ioctl(fd, VIDIOC_QBUF, &buf);
      continue;
    }

    const uint8_t *data = this->cap_mem_[buf.index];
    const uint8_t active = this->raw_active_;
    if (active != 0) {
      RawFrame raw{};
      raw.data = data;
      raw.len = buf.bytesused;
      raw.width = (uint16_t) this->width_;
      raw.height = (uint16_t) this->height_;
      raw.fourcc = this->pixfmt_;
      raw.sequence = ++this->raw_sequence_;
      raw.timestamp_us = esp_timer_get_time();
      for (size_t i = 0; i < this->raw_sinks_.size(); i++) {
        if (active & (1u << i))
          this->raw_sinks_[i]->on_raw_frame(raw);
      }
    }

    int result = 0;
    if (want_jpeg) {
      // Mean luma of the frame, sampled, so brightness can be judged from the
      // log without moving the picture anywhere. YUV 4:2:0 here is packed
      // U Y Y / V Y Y per line, so two of every three bytes are luma.
      if (this->pixfmt_ == V4L2_PIX_FMT_YUV420) {
        uint32_t sum = 0, n = 0;
        for (size_t i = 1; i < buf.bytesused; i += 3 * 97) {
          sum += data[i];
          n++;
        }
        ESP_LOGI(TAG, "frame luma: mean %u of 255 (%u samples)", (unsigned) (n ? sum / n : 0), (unsigned) n);
      }
      // Snapshot who asked before the encode, so a request that arrives during
      // it is answered by the next frame rather than lost.
      const uint8_t requesters = this->single_requesters_ | this->stream_requesters_;
      if (this->encode_(data, buf.bytesused, out)) {
        out->requesters = requesters;
        result = 1;
      } else {
        result = -1;
      }
    }
    if (ioctl(fd, VIDIOC_QBUF, &buf) != 0)
      ESP_LOGE(TAG, "QBUF %u failed (errno %d)", (unsigned) buf.index, errno);
    return result;
  }
}

}  // namespace p4_csi_camera
}  // namespace esphome

#endif  // USE_ESP32_VARIANT_ESP32P4
