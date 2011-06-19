cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# Set more strict compile warnings, the ROS defaults are:
#   -W -Wall -Wno-unused-parameter -fno-strict-aliasing
set(ROS_COMPILE_FLAGS -W -Wall -Wextra -Wno-unused-parameter)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#enable_language(Fortran)
#include(FortranCInterface)
#find_package(LAPACK)
#find_package(BLAS)
#set(MATRIXLIBS ${LAPLACK_LIBRARY} ${BLAS_LIBRARY})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#target_link_libraries(liblapack libblas libm)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
# src/filt.cpp

#rosbuild_add_executable(test_node src/test.cpp src/filtUKF.cpp src/dummy.F)
#target_link_libraries(test_node lapack)
#target_link_libraries(test_node blas) 
#target_link_libraries(test_node m)
#rosbuild_add_executable(nav src/nav.cpp)
rosbuild_add_executable(nav_ekf_ros src/nav_ekf_ros.cpp src/filt.cpp)
#rosbuild_add_executable(nav_ekf_rosREAD src/nav_ekf_rosREAD.cpp src/filt.cpp)
rosbuild_add_executable(nav_ekf src/nav_ekf.cpp src/filt.cpp)
