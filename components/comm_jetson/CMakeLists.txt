idf_component_register(SRCS "comm_jetson.c"
                    INCLUDE_DIRS "include"
                    REQUIRES app_config protocol esp_driver_uart)
