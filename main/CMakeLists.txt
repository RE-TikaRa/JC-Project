idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES app_config protocol comm_jetson motor_driver arm_control)
