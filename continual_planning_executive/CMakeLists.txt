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
    src/continualPlanning.cpp
    src/symbolicState.cpp
    src/plan.cpp
    src/planExecutor.cpp
    src/load_plugins.cpp 
    src/statusPublisher.cpp
    )

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_boost_directories()

#common commands for building c++ executables and libraries
rosbuild_add_library(continual_planning ${planner_executive_SRCS})
rosbuild_add_executable(${PROJECT_NAME} src/main.cpp )
target_link_libraries(continual_planning ${QT_LIBRARIES} )
target_link_libraries(${PROJECT_NAME} continual_planning)
#rosbuild_add_executable(continual_planning_server ${planner_executive_SRCS} src/action_server_main.cpp src/statusPublisher.cpp)
#target_link_libraries(continual_planning_server ${QT_LIBRARIES})
#rosbuild_link_boost(${PROJECT_NAME} thread)

