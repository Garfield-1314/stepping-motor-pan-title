idf_component_register(
    SRCS "main.c"
         "stepper_ledc.c"
         "limit_switch.c"
         "gcode_ledc.c"
         "PCA9546A.c"
         "nvs_params.c"
         "AS5600.c"
         "encoder.c"

    INCLUDE_DIRS "."
    REQUIRES esp_driver_ledc esp_driver_gpio esp_driver_uart esp_driver_i2c nvs_flash
)
