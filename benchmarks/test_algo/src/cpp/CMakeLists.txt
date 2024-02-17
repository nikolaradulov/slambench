cmake_minimum_required(VERSION 2.8)

# CPP version of the testing algorithm for event cameras
set(test_VERSION cpp)

#declare the needed funtions for the cmake file 

FIND_PACKAGE(GLUT)


find_package(OpenGL )
#find_package(Pangolin )
find_package(OpenCV REQUIRED)
#find_package(SuiteSparse REQUIRED) # Apparently needed by g2o
#find_package(G2O REQUIRED)
#find_package(Boost COMPONENTS thread filesystem system REQUIRED)


include_directories(./)
include_directories(
  ./include
  ${EIGEN3_INCLUDE_DIR}
 # ${Pangolin_INCLUDE_DIR}
 # ${G2O_INCLUDE_DIR}
  ./include
 # ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
 # ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
  ${OpenCV_INCLUDE_DIRS}
)

# ------------ COMPILER FLAGS ------------ 
# Also add some useful compiler flag
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -O3 -std=c++11 -g -Wno-error=deprecated-declarations -ftemplate-backtrace-limit=0") 
# Also add some useful compiler flag
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native") 

# -------- SOURCE -------- 

set(SOURCE_FILES
  ./test_library.cpp)

set(CPP_UI_EXTRA_INCLUDE .)

# build shared library.
add_library(test-${test_VERSION} ${SOURCE_FILES} ${CPP_UI_EXTRA_FILES})
SET_TARGET_PROPERTIES(test-${test_VERSION} PROPERTIES COMPILE_FLAGS "-fPIC")
install(TARGETS test-cpp  DESTINATION  lib/)
generate_slam_library(test-${test_VERSION})