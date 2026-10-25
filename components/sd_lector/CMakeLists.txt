idf_component_register(SRCS "sd_lector.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver fatfs sdmmc shared)
