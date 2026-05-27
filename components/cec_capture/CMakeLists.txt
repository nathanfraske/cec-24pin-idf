idf_component_register(
    SRCS
        "cec_capture.c"
        # Source files added during porting:
        # "pretrigger_buffer.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer
)
