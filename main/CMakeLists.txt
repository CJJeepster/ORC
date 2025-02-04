set(srcs "IMU_fusion_int.c")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")
