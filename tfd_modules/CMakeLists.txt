cmake_minimum_required(VERSION 2.8.3)
project(tfd_modules)

option(NO_ROS "Build without ROS" false)
if (NO_ROS)
    include(no_ros.cmake)
else()
    include(ros.cmake)
endif()
