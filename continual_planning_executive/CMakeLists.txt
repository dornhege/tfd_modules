cmake_minimum_required(VERSION 2.8.3)
project(continual_planning_executive)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
pluginlib
actionlib
continual_planning_msgs
)

find_package(Qt4 REQUIRED)
set( QT_USE_QTGUI TRUE )
set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})

# needed: otherwise boost signals name clash
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

catkin_package(
INCLUDE_DIRS include
LIBRARIES continual_planning
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(planner_executive_SRCS
    src/continualPlanning.cpp
    src/symbolicState.cpp
    src/plan.cpp
    src/planExecutor.cpp
    src/load_plugins.cpp 
    src/statusPublisher.cpp
    )


#common commands for building c++ executables and libraries
add_library(continual_planning ${planner_executive_SRCS})
add_executable(${PROJECT_NAME} src/main.cpp )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(continual_planning ${catkin_EXPORTED_TARGETS})
add_dependencies(continual_planning continual_planning_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} continual_planning_msgs_generate_messages_cpp)

target_link_libraries(continual_planning ${catkin_LIBRARIES} ${QT_LIBRARIES} )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} continual_planning)


#rosbuild_add_executable(continual_planning_server ${planner_executive_SRCS} src/action_server_main.cpp src/statusPublisher.cpp)
#target_link_libraries(continual_planning_server ${QT_LIBRARIES})
#rosbuild_link_boost(${PROJECT_NAME} thread)

