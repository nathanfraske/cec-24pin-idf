idf_component_register(
    SRCS
        "ina226.c"
        "cec_adc.c"
        # Source files added during porting:
        # "acs712.c"
        # "thermistor.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_i2c
        esp_adc
)
