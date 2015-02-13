cmake_minimum_required(VERSION 2.8.3)
project(continual_planning_monitor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    continual_planning_msgs
)

catkin_package(
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt4 REQUIRED)
set( QT_USE_QTGUI TRUE )
set( QT_USE_QTOPENGL TRUE )
set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})

QT4_WRAP_UI(uis_h src/ContinualPlanningMonitorWindow.ui)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# include this for ui_h
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${catkin_INCLUDE_DIRS})

set(qt_srcs
      src/ContinualPlanningMonitorWindow.cpp)
set(qt_hdrs
      src/ContinualPlanningMonitorWindow.h)

qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})


add_executable(${PROJECT_NAME} src/main.cpp ${uis_h} ${qt_srcs} ${qt_moc_srcs})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} continual_planning_msgs_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} boost_system)

