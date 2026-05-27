idf_component_register(
    SRCS
        "cec_filters.c"
        "cec_state.c"
        # Source files added during porting:
        # "swing_detector.c"
        # "adaptive_transient.c"
        # "profile.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash
)
