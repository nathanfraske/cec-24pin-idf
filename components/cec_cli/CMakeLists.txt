idf_component_register(
    SRCS
        "cec_cli.c"
    INCLUDE_DIRS "include"
    REQUIRES
        driver
        esp_driver_usb_serial_jtag
        vfs
)
