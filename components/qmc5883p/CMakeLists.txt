idf_component_register(SRCS "qmc5883p.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_i2c esp_timer freertos)
