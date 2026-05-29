idf_component_register(
    SRCS
        "ina226.c"
        # cec_adc + thermistor stay in the build for the NTC channel that
        # arrives with the daughterboard; unused at runtime in v2.
        "cec_adc.c"
        "thermistor.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_adc
)
