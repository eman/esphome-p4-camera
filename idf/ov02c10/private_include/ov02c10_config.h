/*
 * Build configuration for the OV02C10 driver.
 *
 * Upstream, these are Kconfig symbols from esp_cam_sensor's menuconfig
 * (sensors/ov02c10/Kconfig.ov02c10 in the pull request this driver comes
 * from). This component is built out of that tree, so the values are fixed
 * here instead, chosen for the Guition JC1060P470's module.
 */

#pragma once

/* Register through the .esp_cam_sensor_detect_fn section so esp_video's
 * auto-detect walk finds the sensor at 0x36 on the MIPI-CSI port. */
#define CONFIG_CAMERA_OV02C10_AUTO_DETECT_MIPI_INTERFACE_SENSOR 1

/* Default output: index 2 of the format table, 1920x1080 RAW10 at 30 fps
 * over two MIPI lanes. The panel wires both lanes. */
#define CONFIG_CAMERA_OV02C10_MIPI_IF_FORMAT_INDEX_DAFAULT 2

/* Send MIPI line start/end short packets. The register tables set 0x4800 to
 * 0x64 accordingly, and the ISP is told to expect them: the two must agree,
 * or the ISP counts lines wrongly and raises "hnum / vnum setting error" on
 * every frame. */
#define CONFIG_CAMERA_OV02C10_CSI_LINESYNC_ENABLE 1

/* The panel's module is fixed focus: no VCM, so do not drive the pad-out
 * registers the upstream driver uses for a lens motor. */
#define CONFIG_CAMERA_OV02C10_ISP_AF_ENABLE 0

/* Gain ceiling handed to the auto-exposure algorithm, x1000. */
#define CONFIG_CAMERA_OV02C10_ABSOLUTE_GAIN_LIMIT 66016

/* Prefer analogue gain over digital: less noise for the same brightness. */
#define CONFIG_CAMERA_OV02C10_ANA_GAIN_PRIORITY 1
#define CONFIG_CAMERA_OV02C10_DIG_GAIN_PRIORITY 0

#define CONFIG_CAMERA_OV02C10_MAX_SUPPORT 1
