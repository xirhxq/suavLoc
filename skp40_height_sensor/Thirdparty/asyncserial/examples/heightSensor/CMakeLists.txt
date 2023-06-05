##########################################################################
cmake_minimum_required(VERSION 3.10)
##########################################################################
set(SERIAL_DEMO_TARGET skp40_height_library)
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    include
)
##########################################################################
message("DEBUG IN HeightSensor")
message(${PROJECT_SOURCE_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(${SERIAL_DEMO_TARGET} SHARED src/skp40_height_sensor.cpp include/skp40_height_sensor.h)
target_link_libraries(${SERIAL_DEMO_TARGET} asyncserial)
##########################################################################
add_executable(${SERIAL_DEMO_TARGET}_test src/test_node.cpp)
target_link_libraries(${SERIAL_DEMO_TARGET}_test ${SERIAL_DEMO_TARGET})
##########################################################################
