idf_component_register(
    SRCS
        # Source files added during porting:
        # "burst_capture.c"
        # "pretrigger_buffer.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer
)
