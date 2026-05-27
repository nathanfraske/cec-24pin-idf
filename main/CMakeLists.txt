idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        # CEC components
        cec_sensors
        cec_detection
        cec_capture
        # ESP-IDF components used directly by main.c
        esp_timer
        esp_system
        esp_psram
        spi_flash
        esp_driver_i2c
)
