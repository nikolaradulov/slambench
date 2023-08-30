cmake_minimum_required(3.9)
project(slam_eds)


SET(SLAMBENCH_C_WRAPPER	-Wl,--whole-archive -L${WRAPPER_PATH} slambench-c-wrapper	-Wl,--no-whole-archive)
SET(NULL_FILE_PATH	${WRAPPER_PATH}/null.cpp)
SET(EDS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eds)
SET(EDS_BUILD_DIR ${EDS_DIR}/build)

# build eds
add_subdirectory(${EDS_DIR} ${EDS_BUILD_DIR})


# generate_slam_library(${appname}-original)