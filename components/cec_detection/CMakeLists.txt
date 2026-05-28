idf_component_register(
    SRCS
        "cec_filters.c"
        "cec_state.c"
        "cec_layer1.c"
        "cec_layer2.c"
        "cec_layer3.c"
        "cec_swing.c"
        "cec_nvs.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash
)
