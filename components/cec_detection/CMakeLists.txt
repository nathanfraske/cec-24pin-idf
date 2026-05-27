idf_component_register(
    SRCS
        # Source files added during porting:
        # "state_classifier.c"
        # "swing_detector.c"
        # "adaptive_transient.c"
        # "profile.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash
)
