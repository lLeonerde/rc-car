idf_component_register(SRCS "rc_car.c" "hello_world_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt nvs_flash driver)