idf_component_register(
    SRCS
        "ina226.c"
        "cec_adc.c"
        "thermistor.c"
        "acs712.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_adc
)
