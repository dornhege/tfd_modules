cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

set(opl_RoomScanning_SOURCES
   src/Door.cpp
   src/Pose.cpp
   src/Robot.cpp
   src/State.cpp
   src/StateFactory.cpp
   src/Target.cpp
   src/Pose_isReachableFrom.cpp
   src/Pose_isReachableFrom_plannerCall.cpp
   src/Robot_driveDuration.cpp
   src/Robot_driveDuration_plannerCall.cpp)


#common commands for building c++ executables and libraries
rosbuild_add_library(tfd_opl_RoomScanning ${opl_RoomScanning_SOURCES})
target_link_libraries(tfd_opl_RoomScanning tfd_opl_interface)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example src/main.cpp)
#target_link_libraries(example ${PROJECT_NAME})
