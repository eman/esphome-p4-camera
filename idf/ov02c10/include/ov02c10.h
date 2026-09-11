/*
* SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
*
* SPDX-License-Identifier: Apache-2.0
*/
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_cam_sensor_types.h"

#define OV02C10_SCCB_ADDR   0x36

/**
* @brief Power on camera sensor device and detect the device connected to the designated sccb bus.
*
* @param[in] config Configuration related to device power-on and detection.
* @return
*      - Camera device handle on success, otherwise, failed.
*/
esp_cam_sensor_device_t *ov02c10_detect(esp_cam_sensor_config_t *config);

/**
 * @brief Look up one of the driver's built-in output formats, for VIDIOC_S_SENSOR_FMT.
 *
 * @param[in] index Position in the driver's format table: 0 = 1288x728 one lane,
 *                  1 = 1920x1080 one lane, 2 = 1920x1080 two lanes (the default).
 * @return The format, or NULL if the index is out of range.
 */
const esp_cam_sensor_format_t *ov02c10_get_format_by_index(size_t index);

#ifdef __cplusplus
}
#endif
