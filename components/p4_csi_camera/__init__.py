"""The panel's OV02C10 camera over MIPI-CSI, as a Home Assistant camera entity.

Upstream ESPHome has no CSI support: `esp32_camera` is DVP-only and will not
build for the ESP32-P4. The pipeline underneath is Espressif's `esp_video`,
which presents the CSI controller, the ISP and the hardware JPEG encoder as
V4L2 devices (/dev/video0, /dev/video20, /dev/video10), and `esp_cam_sensor`
for the sensor itself.

Three things this board forces, all of which esp_video supports directly:

  * The sensor is an OV02C10 at 0x36 - confirmed from its chip id, not from the
    address - and esp_cam_sensor 2.4 ships no driver for it. `idf/ov02c10` is
    Espressif's own driver from esp-video-components pull request #46, built
    here as an ESP-IDF component so that its ISP tuning file is compiled in.
  * The SCCB shares the I2C bus ESPHome already owns for the GT911 touch
    controller, so the bus handle is handed over rather than letting esp_video
    create a second master on the same pins.
  * The 2.5V rail the CSI PHY needs is already up, because the DSI PHY runs on
    it and `esp_ldo` channel 3 claimed it at boot. `dont_init_ldo` stops the two
    fighting over the channel.

The component implements ESPHome's `camera::Camera` interface, so Home
Assistant sees a camera entity and can pull stills or a stream over the native
API. Nothing touches the CSI pipeline until the first request.
"""

from pathlib import Path

import esphome.codegen as cg
from esphome.components import esp32, i2c
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_NAME, CONF_RESOLUTION
from esphome.core.entity_helpers import setup_entity

CODEOWNERS = ["@emmanuel"]
DEPENDENCIES = ["esp32", "i2c"]
AUTO_LOAD = ["camera"]

CONF_RESET_PIN = "reset_pin"
CONF_PWDN_PIN = "pwdn_pin"
CONF_INIT_LDO = "init_ldo"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_MAX_FRAMERATE = "max_framerate"
CONF_SETTLE_FRAMES = "settle_frames"
CONF_HORIZONTAL_MIRROR = "horizontal_mirror"
CONF_VERTICAL_FLIP = "vertical_flip"

p4_csi_camera_ns = cg.esphome_ns.namespace("p4_csi_camera")
P4CsiCamera = p4_csi_camera_ns.class_("P4CsiCamera", cg.Component)
SensorMode = p4_csi_camera_ns.enum("SensorMode")

# Index into the driver's format table, see idf/ov02c10/ov02c10.c.
RESOLUTIONS = {
    "1920x1080": SensorMode.SENSOR_MODE_1920X1080,
    "1288x728": SensorMode.SENSOR_MODE_1288X728,
}

# The driver lives outside `components/` on purpose: ESPHome compiles every
# source file it finds under a component directory, and this one must be built
# once, by ESP-IDF, or its symbols are defined twice.
IDF_COMPONENT_PATH = Path(__file__).resolve().parent.parent.parent / "idf" / "ov02c10"


def _p4_only(config):
    variant = esp32.get_esp32_variant()
    if variant != esp32.const.VARIANT_ESP32P4:
        raise cv.Invalid(f"p4_csi_camera needs an ESP32-P4; this is an {variant}")
    return config


CONFIG_SCHEMA = cv.All(
    cv.ENTITY_BASE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(P4CsiCamera),
            cv.Required(CONF_NAME): cv.string,
            # The SCCB rides the bus ESPHome already has on these pins.
            cv.GenerateID(i2c.CONF_I2C_ID): cv.use_id(i2c.I2CBus),
            # Raw GPIO numbers, not pin schemas: esp_video takes gpio_num_t and
            # configures these itself. -1 means "not wired", which is the case
            # on this panel: a GPIO sweep found nothing that changes how the
            # sensor answers.
            cv.Optional(CONF_RESET_PIN, default=-1): cv.int_range(min=-1, max=56),
            cv.Optional(CONF_PWDN_PIN, default=-1): cv.int_range(min=-1, max=56),
            # Off, because the DSI PHY already brought this rail up.
            cv.Optional(CONF_INIT_LDO, default=False): cv.boolean,
            # Hardware encoder quality, 1..100. 1080p lands around 200 KB at 30
            # and 600 KB at 50; every byte crosses the ESP-Hosted Wi-Fi link.
            cv.Optional(CONF_JPEG_QUALITY, default=30): cv.int_range(min=1, max=100),
            cv.Optional(CONF_RESOLUTION, default="1920x1080"): cv.enum(RESOLUTIONS, lower=True),
            # Ceiling on frames handed to the API while streaming.
            cv.Optional(CONF_MAX_FRAMERATE, default="3 fps"): cv.framerate,
            # Frames discarded after the stream starts, so auto exposure has
            # settled before the first one is kept. ~33 ms each. The ISP
            # pipeline restarts exposure and gain from its defaults every time
            # the stream starts, and in a dim room it needs the better part of
            # a second to climb to the right gain.
            cv.Optional(CONF_SETTLE_FRAMES, default=24): cv.int_range(min=0, max=90),
            # Sensor-side mirror and flip, for however the module is mounted.
            cv.Optional(CONF_HORIZONTAL_MIRROR, default=False): cv.boolean,
            cv.Optional(CONF_VERTICAL_FLIP, default=False): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _p4_only,
)


async def to_code(config):
    cg.add_define("USE_CAMERA")
    var = cg.new_Pvariable(config[CONF_ID])
    await setup_entity(var, config, "camera")
    await cg.register_component(var, config)
    cg.add(var.set_i2c_bus(await cg.get_variable(config[i2c.CONF_I2C_ID])))
    cg.add(var.set_reset_pin(config[CONF_RESET_PIN]))
    cg.add(var.set_pwdn_pin(config[CONF_PWDN_PIN]))
    cg.add(var.set_init_ldo(config[CONF_INIT_LDO]))
    cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    cg.add(var.set_sensor_mode(config[CONF_RESOLUTION]))
    cg.add(var.set_max_update_interval(int(1000 / config[CONF_MAX_FRAMERATE])))
    cg.add(var.set_settle_frames(config[CONF_SETTLE_FRAMES]))
    cg.add(var.set_mirror(config[CONF_HORIZONTAL_MIRROR], config[CONF_VERTICAL_FLIP]))

    esp32.add_idf_component(name="espressif/esp_video", ref="2.4.1")
    # A local-path component is named after its directory by the component
    # manager, and that is the name ESPHome's src component REQUIRES, so the
    # two must agree.
    esp32.add_idf_component(name=IDF_COMPONENT_PATH.name, path=str(IDF_COMPONENT_PATH))

    # ESPHome builds IDF with CONFIG_LOG_MAXIMUM_LEVEL=1 (ERROR), which compiles
    # every ESP_LOGI/D/W in every IDF component out of existence - including the
    # sensor driver's and the CSI and ISP layers'. esp_log_level_set cannot
    # bring back calls that were never emitted, so bring-up is blind: a failing
    # capture looks like nothing happening.
    #
    # INFO, not DEBUG. DEBUG also compiles in ESP-Hosted's SDIO chatter, which
    # emitted 20,000+ lines in two minutes and swamped the API log stream so
    # badly that captures appeared to fail when only their log lines were being
    # dropped. Its logging does not honour esp_log_level_set, so the only lever
    # is not compiling it in. A choice symbol: setting the derived
    # CONFIG_LOG_MAXIMUM_LEVEL int does nothing, the named option is what counts.
    esp32.add_idf_sdkconfig_option("CONFIG_LOG_MAXIMUM_LEVEL_INFO", True)

    esp32.add_idf_sdkconfig_option("CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE", True)
    esp32.add_idf_sdkconfig_option("CONFIG_ESP_VIDEO_ENABLE_ISP_VIDEO_DEVICE", True)
    esp32.add_idf_sdkconfig_option("CONFIG_ESP_VIDEO_ENABLE_HW_JPEG_ENC_VIDEO_DEVICE", True)
    # The "isp_task": reads the ISP's AE/AWB/histogram statistics every frame
    # and drives exposure, gain, white balance, CCM and gamma from the sensor's
    # tuning file. Without it every frame is at the sensor's power-on exposure
    # with no white balance.
    esp32.add_idf_sdkconfig_option("CONFIG_ESP_VIDEO_ENABLE_ISP_PIPELINE_CONTROLLER", True)
