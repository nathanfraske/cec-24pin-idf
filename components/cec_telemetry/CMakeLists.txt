idf_component_register(
    SRCS
        "cec_teleplot.c"
    INCLUDE_DIRS "include"
    REQUIRES
        driver
        esp_driver_uart
)
