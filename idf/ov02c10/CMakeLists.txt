# OV02C10 sensor driver for esp_cam_sensor, built as an ESP-IDF component.
#
# This is a component rather than ESPHome source because of the ISP tuning
# file: esp_ipa compiles cfg/ov02c10_default.json into the firmware only when
# a component's project_include.cmake appends it to ESP_IPA_JSON_CONFIG_FILE_PATH,
# and only ESP-IDF components get a project_include.cmake.

idf_component_register(SRCS "ov02c10.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "private_include"
                       REQUIRES esp_cam_sensor esp_sccb_intf
                       PRIV_REQUIRES esp_driver_gpio)

# Nothing references this driver by symbol - esp_video finds it through the
# .esp_cam_sensor_detect_fn section - so without this the linker never pulls
# the object out of the archive, and the sensor is never detected. This is the
# same line esp_cam_sensor's own CMakeLists uses for every built-in sensor.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u ov02c10_detect")
