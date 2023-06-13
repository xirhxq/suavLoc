##########################################################################
cmake_minimum_required(VERSION 3.10)
##########################################################################
project("asyncserial")
set(CMAKE_BUILD_TYPE "Debug")
##########################################################################
option(BUILD_EXAMPLES "Build all examples provided with this library" OFF)
##########################################################################
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(${PROJECT_NAME} SHARED src/AsyncSerial.cpp)
##########################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Werror -Wextra -Wpedantic)
endif()
##########################################################################
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} boost_system boost_thread pthread)
##########################################################################
if(BUILD_EXAMPLES)
  add_subdirectory(examples/async-serial-receive)
  add_subdirectory(examples/sync-serial-transmit)
endif()

##########################################################################
