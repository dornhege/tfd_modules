cmake_minimum_required(VERSION 2.8.3)
project(simple_modules)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tfd_modules
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_modules
#  CATKIN_DEPENDS roscpp tfd_modules
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(testModule src/testModule.cpp)
add_library(crewPlanningModule src/crewPlanning.cpp)
add_library(transportModule src/transport.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(simple_modules_node
#   ${catkin_LIBRARIES}
# )

