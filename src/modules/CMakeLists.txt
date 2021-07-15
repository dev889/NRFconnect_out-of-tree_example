cmake_minimum_required(VERSION 3.13.1)

add_subdirectory_ifdef(CONFIG_VL53LX ${PROJECT_SOURCE_DIR}/src/modules/vl53lx)