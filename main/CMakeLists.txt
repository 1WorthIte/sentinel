idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_timer esp_driver_ledc esp_driver_gpio esp_driver_i2c esp_adc esp_rom)