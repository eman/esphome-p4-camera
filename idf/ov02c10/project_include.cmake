# Hand esp_ipa the ISP tuning for this sensor. esp_ipa's CMakeLists collects
# every path on this property and generates esp_video_ipa_config.c from them;
# esp_video looks the table up by sensor name ("OV02C10") when it starts the
# ISP pipeline, and runs auto exposure, white balance, colour correction and
# gamma from it. Without an entry the pipeline is simply not started and the
# picture is raw ISP defaults: fixed exposure, no white balance.
idf_build_set_property(ESP_IPA_JSON_CONFIG_FILE_PATH
                       "${CMAKE_CURRENT_LIST_DIR}/cfg/ov02c10_default.json" APPEND)
