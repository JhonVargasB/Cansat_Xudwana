idf_component_register(SRCS "buzzer.c"
                    INCLUDE_DIRS "include"
                    REQUIRES shared esp_driver_gpio esp_driver_gptimer freertos)
