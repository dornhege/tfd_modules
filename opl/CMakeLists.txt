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

find_package(Qt4 REQUIRED)
#set( QT_USE_QTGUI TRUE )
#set( QT_USE_QTOPENGL TRUE )
#set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
include_directories(${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/src/parsing)
set(opl_SOURCES
   src/Domain.cpp
   src/DurativeAction.cpp
   src/Element.cpp
   src/Fluent.cpp
   src/Function.cpp
   src/Initialization.cpp
   src/InterfaceGenerator.cpp
   src/KeyWordList.cpp
   src/Module.cpp
   src/NamedList.cpp
   src/Number.cpp
   src/Object.cpp
   src/OPLSemantic.cpp
   src/Problem.cpp
   src/StringConstant.cpp
   src/stringutil.cpp
   src/SymbolAccess.cpp
   src/SymbolDefinition.cpp
   src/Type.cpp
   src/parsing/DomainStructure.cpp
   src/parsing/DurativeActionStructure.cpp
   src/parsing/ElementDetector.cpp
   src/parsing/FluentStructure.cpp
   src/parsing/FunctionStructure.cpp
   src/parsing/InitializationStructure.cpp
   src/parsing/ModuleStructure.cpp
   src/parsing/NamedListStructure.cpp
   src/parsing/NumberStructure.cpp
   src/parsing/ObjectStructure.cpp
   src/parsing/Parser.cpp
   src/parsing/ParseUnit.cpp
   src/parsing/ProblemStructure.cpp
   src/parsing/StringStructure.cpp
   src/parsing/SymbolAccessStructure.cpp
   src/parsing/SymbolDefinitionStructure.cpp
   src/parsing/Translator.cpp
   src/parsing/TypeStructure.cpp)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(opl ${opl_SOURCES})
rosbuild_add_executable(opl_translate_domain src/opl_translate_domain.cpp)
target_link_libraries(opl_translate_domain opl dl ${QT_LIBRARIES})
rosbuild_add_executable(opl_translate_problem src/opl_translate_problem.cpp)
target_link_libraries(opl_translate_problem opl dl ${QT_LIBRARIES})

