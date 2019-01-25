cmake_minimum_required(VERSION 3.10)
project(reactionWheel)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} "/home/arms/build/dxl_sdk_linux_v1_01/include" "/usr/local/natinst/nidaqmxbase/include")
link_directories("/usr/local/natinst/nidaqmxbase/lib64/")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native -mtune=native -fpermissive -std=c++11 -g")

add_executable(reactionWheel main.cpp pid.cpp EKF.cpp EKF.h)
target_link_libraries(${PROJECT_NAME}  nidaqmxbase nidaqmxbaselv)
