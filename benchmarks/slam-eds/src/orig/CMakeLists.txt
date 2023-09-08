cmake_minimum_required(VERSION 2.8.8)
project(slam_eds)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

find_package(OpenCV REQUIRED)
find_package(Rock)
#rock_init()
#rock_opencv_autodetect(OPENCV_PACKAGE)

SET(EDS_INCLUDE_PATH ${DEPS_PATH}/include/eds)
SET(EDS_SHARED ${DEPS_PATH}/lib/libeds.so)
SET(OpenCV2_INCLUDE_DIR /home/nrad/slambench/deps/opencv/include)
SET(ROCK_BASE_INCLUDE ${DEPS_PATH}/include/base)
SET(TASK_INCLUDE ${NULL_FILE_PATH}/task)
SET(FRAME_INCLUDE ${DEPS_PATH}/include)

add_library(${PROJECT_NAME}-original SHARED slam_eds_library.cpp)

message(${FRAME_INCLUDE})

include_directories(${PROJECT_NAME}-original PUBLIC ${EDS_INCLUDE_PATH} 
    ${SLAMBENCH_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR} 
    ${OpenCV_INCLUDE_DIR} 
    ${OpenCV2_INCLUDE_DIR} 
    ${ROCK_BASE_INCLUDE} 
    ${TASK_INCLUDE} 
    ${FRAME_INCLUDE}
    )
target_link_libraries(${PROJECT_NAME}-original PUBLIC ${EDS_SHARED})

install(TARGETS ${PROJECT_NAME}-original  DESTINATION  /lib)
SET(SLAMBENCH_C_WRAPPER	-Wl,--whole-archive -L${WRAPPER_PATH} slambench-c-wrapper	-Wl,--no-whole-archive)
SET(NULL_FILE_PATH	${NULL_PATH}/null.cpp)

function(generate_slam_library library_name )

    add_library(${library_name}-library SHARED ${NULL_FILE_PATH})
    target_link_libraries(${library_name}-library PRIVATE ${SLAMBENCH_C_WRAPPER} ${library_name})
    SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES LINK_FLAGS "${ARGN}")
    SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES INSTALL_RPATH ${DEPS_PATH}/lib:${WRAPPER_PATH})
    install(TARGETS ${library_name}-library  DESTINATION /lib)

endfunction()

generate_slam_library(${PROJECT_NAME}-original "-Wl,--disable-new-dtags")