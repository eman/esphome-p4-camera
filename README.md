# esphome-p4-camera

An [ESPHome](https://esphome.io) camera for the ESP32-P4's MIPI-CSI interface,
and a sensor driver for the OmniVision OV02C10 that Espressif's `esp_cam_sensor`
does not yet ship.

Developed on the Guition JC1060P470 (7" ESP32-P4 panel with an OV02C10 camera
module). Upstream ESPHome has no CSI support: `esp32_camera` is DVP-only and
does not build for the P4. This component sits on Espressif's `esp_video`,
which presents the CSI controller,
the ISP and the hardware JPEG encoder as V4L2 devices, and implements ESPHome's
`camera::Camera`, so Home Assistant gets a camera entity over the native API
with no further configuration.

For smooth video, pair it with
[`esphome-rtsp-h264`](https://github.com/eman/esphome-rtsp-h264), which takes
raw frames from this component and serves hardware-encoded H.264 over RTSP.

## Configuration

```yaml
external_components:
  - source: github://eman/esphome-p4-camera

i2c:
  - id: bus_a
    sda: GPIO07
    scl: GPIO08
    frequency: 400kHz

p4_csi_camera:
  id: cam
  name: Camera
  i2c_id: bus_a              # the bus carrying the sensor's SCCB
  reset_pin: -1              # GPIO numbers, or -1 when not wired (the JC1060P470)
  pwdn_pin: -1
  init_ldo: false            # true if nothing else brings up the 2.5 V MIPI PHY rail
  jpeg_quality: 50           # 1..100; 1080p is ~110 KB at 50
  resolution: 1920x1080      # or 1288x728 (one MIPI lane, Guition's own choice)
  max_framerate: 3 fps       # ceiling on frames handed to the API while streaming
  settle_frames: 24          # frames given to auto exposure before the first still
  horizontal_mirror: false
  vertical_flip: false
```

The full example, with the board's PSRAM and I²C, is in `example/camera.yaml`.

`tools/camtest.py` exercises the entity the way Home Assistant does: stills and
a stream over the native API, with the JPEGs written out to look at.

Two diagnostics are exposed for lambdas: `probe()` lists the V4L2 devices, and
`gain_sweep()` pauses auto exposure, writes exposure and gain registers
straight to the sensor and logs the frame's mean luma at each step. That sweep
is how the gain register finding above was made; the session-close log line
reports where auto exposure landed and reads the sensor's registers back.

## What is in here

```
components/p4_csi_camera/   the ESPHome component
  raw_video.h               the raw-frame interface other components consume
idf/ov02c10/                the OV02C10 driver, as an ESP-IDF component
  cfg/ov02c10_default.json  ISP tuning: auto exposure, white balance, colour
```

### The sensor driver

`idf/ov02c10` is Espressif's own OV02C10 driver from
[esp-video-components pull request #46](https://github.com/espressif/esp-video-components/pull/46)
(Apache-2.0), with fixes found on the bench, each marked in place:

- the grouped exposure/gain update read a field esp_video leaves at zero, so
  exposure sat at the 8-line minimum whatever the light;
- the gain table was not this sensor's, and neither, on this module, is the
  Linux driver's analogue gain register: written in every encoding and
  mode-register setting tried, `0x3508:0x3509` changes nothing in the
  picture, while `0x350a:0x350b` (the Linux driver's "digital gain") does.
  The table is regenerated to drive that register from 1× to 15.9×; upstream
  declared 63× and delivered about 4×;
- the two-lane mode allows exposures up to 64 ms: the sensor stretches the
  frame when exposure exceeds it, so a dim room drops to 15 fps rather than
  going black, and bright scenes stay at 30 fps;
- the two-lane 1080p timing entry lists what its own register table programs;
- `tline_ns` is filled in, without which the auto-gain algorithm sees an
  exposure range of 0..0 and esp_video refuses to start;
- mode tables no longer end with "stream on", so loading a mode does not start
  the sensor.

It is built as an ESP-IDF component rather than as ESPHome source so that its
ISP tuning file is compiled in: `esp_ipa` collects those through a component's
`project_include.cmake`. With it, esp_video runs auto exposure, white balance,
colour correction and gamma. The tuning file's white-balance window was
widened from the upstream daylight-only window, metering switched from
highlight priority (a ceiling lamp set the exposure) to low-light priority,
and the colour matrix set to identity, which measured most neutral on this
module.

### The capture path

The main loop only ever sets request flags and hands finished JPEGs to the
API. A task on the second core owns the V4L2 session: it opens the pipeline on
the first request, closes it six seconds after the last, grabs frames only when
the loop asks for one, and drops an image that an API client has held for more
than four seconds so one stalled connection cannot wedge the camera.

Frames are captured as YUV 4:2:0, which the hardware H.264 encoder needs. On
ESP32-P4 silicon before revision 3 the hardware JPEG encoder does not accept
it, so stills go through the pixel processing accelerator to RGB565 first;
this is automatic.

Raw frames are available to other components through `RawVideoSink`
(`raw_video.h`): register a sink, ask for streaming, and every frame the sensor
produces is delivered on the capture task while the request stands.

## Two things that cost days, recorded so they do not cost yours

- **ESP-IDF 5.5.5's ISP driver logs its error interrupts from inside the
  interrupt handler.** A frame that trips one continuously becomes an
  interrupt storm that the interrupt watchdog ends 800 ms later, and nothing
  reaches the API log stream because panics only go to the serial console.
  esp_video masks those interrupts on IDF ≥ 5.5.6; this component masks them
  after `STREAMON` and reports the raw status when the session closes.
- **ESPHome builds IDF without runtime log-level control**, so
  `esp_log_level_set()` is a no-op. Hold the `esp-idf` logger tag at INFO in
  your `logger:` block or the tuning library's debug output will flood the
  console at 30 fps.

## Requirements

ESP32-P4, ESP-IDF 5.5, ESPHome 2026.8 or later, PSRAM. `espressif/esp_video`
2.4.1 is added to the build automatically.

## Licence

MIT for this repository's own code. `idf/ov02c10` is Apache-2.0, as its
headers state.
