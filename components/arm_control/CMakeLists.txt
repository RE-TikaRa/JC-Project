idf_component_register(SRCS "arm_control.c"
                    INCLUDE_DIRS "include"
                    REQUIRES app_config protocol kinematics motor_driver comm_jetson)
