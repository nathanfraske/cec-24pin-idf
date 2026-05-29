idf_component_register(
    SRCS
        "cec_capture.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_timer
        cec_telemetry
)
