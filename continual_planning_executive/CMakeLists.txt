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

find_package(Qt4 REQUIRED)
set( QT_USE_QTGUI TRUE )
set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})

# needed: otherwise boost signals name clash
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(planner_executive_SRCS
   src/symbolicState.cpp
   src/plan.cpp
   src/planParser.cpp
   src/plannerInterface.cpp
   src/plannerTFDM.cpp
   src/planExecutorInterface.cpp
   src/planExecutorROSNavigation.cpp
   src/actionExecutorROSNavigation.cpp
   src/domainParser.cpp)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_boost_directories()

#common commands for building c++ executables and libraries
rosbuild_add_executable(${PROJECT_NAME} ${planner_executive_SRCS} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

