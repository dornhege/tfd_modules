cmake_minimum_required(VERSION 2.8.3)
project(tfd_modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
pluginlib
continual_planning_executive
continual_planning_msgs
)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include module_api ${catkin_INCLUDE_DIRS})

add_definitions(-DROS_BUILD)

catkin_package(
CATKIN_DEPENDS continual_planning_executive
INCLUDE_DIRS include module_api)

set(module_api_SOURCES
   module_api/moduleLoader.cpp
   module_api/moduleLoaderDLL.cpp
   module_api/moduleLoaderLDL.cpp
   module_api/pddlModuleLoader.cpp
   module_api/pddlModuleLoaderDLL.cpp
   module_api/pddlModuleLoaderLDL.cpp
   module_api/printTypes.cpp)

set(module_api_HEADERS
   module_api/moduleLoaderDLL.h
   module_api/moduleLoader.h
   module_api/moduleLoaderLDL.h
   module_api/pddlModuleLoaderDLL.h
   module_api/pddlModuleLoader.h
   module_api/pddlModuleLoaderLDL.h
   module_api/printTypes.h)
 
set(preprocess_HEADERS
   downward/preprocess/axiom.h
   downward/preprocess/causal_graph.h
   downward/preprocess/domain_transition_graph_func.h
   downward/preprocess/domain_transition_graph.h
   downward/preprocess/domain_transition_graph_subterm.h
   downward/preprocess/domain_transition_graph_symb.h
   downward/preprocess/helper_functions.h
   downward/preprocess/max_dag.h
   downward/preprocess/module.h
   downward/preprocess/operator.h
   downward/preprocess/scc.h
   downward/preprocess/state.h
   downward/preprocess/successor_generator.h
   downward/preprocess/variable.h)

set(preprocess_SOURCES
   downward/preprocess/axiom.cpp
   downward/preprocess/causal_graph.cpp
   downward/preprocess/domain_transition_graph.cpp
   downward/preprocess/domain_transition_graph_func.cpp
   downward/preprocess/domain_transition_graph_subterm.cpp
   downward/preprocess/domain_transition_graph_symb.cpp
   downward/preprocess/helper_functions.cpp
   downward/preprocess/max_dag.cpp
   downward/preprocess/module.cpp
   downward/preprocess/operator.cpp
   downward/preprocess/planner.cpp
   downward/preprocess/scc.cpp
   downward/preprocess/state.cpp
   downward/preprocess/successor_generator.cpp
   downward/preprocess/variable.cpp)

set(search_HEADERS
   downward/search/axioms.h
   downward/search/best_first_search.h
   downward/search/causal_graph.h
   downward/search/closed_list.h
   downward/search/cyclic_cg_heuristic.h
   downward/search/domain_transition_graph.h
   downward/search/globals.h
   downward/search/plannerParameters.h
   downward/search/heuristic.h
   downward/search/module.h
   downward/search/monitoring.h
   downward/search/no_heuristic.h
   downward/search/greedy_apply_heuristic.h
   downward/search/operator.h
   downward/search/partial_order_lifter.h
   downward/search/scheduler.h
   downward/search/search_engine.h
   downward/search/search_statistics.h
   downward/search/timing.h
   downward/search/state.h
   downward/search/successor_generator.h)

set(search_SOURCES
   downward/search/axioms.cpp
   downward/search/best_first_search.cpp
   downward/search/causal_graph.cpp
   downward/search/closed_list.cpp
   downward/search/cyclic_cg_heuristic.cpp
   downward/search/domain_transition_graph.cpp
   downward/search/globals.cpp
   downward/search/plannerParameters.cpp
   downward/search/heuristic.cpp
   downward/search/module.cpp
   downward/search/monitoring.cpp
   downward/search/no_heuristic.cpp
   downward/search/greedy_apply_heuristic.cpp
   downward/search/operator.cpp
   downward/search/partial_order_lifter.cpp
   downward/search/planner.cpp
   downward/search/scheduler.cpp
   downward/search/search_engine.cpp
   downward/search/search_statistics.cpp
   downward/search/timing.cpp
   downward/search/analysis.cpp
   downward/search/state.cpp
   downward/search/successor_generator.cpp)

set(opl_interface_SOURCES
   downward/opl/interface/AbstractState.cpp
   downward/opl/interface/AbstractStateFactory.cpp
   downward/opl/interface/ObjectLookupTable.cpp
   downward/opl/interface/oplObject.cpp
   downward/opl/interface/FluentMapping.cpp
   downward/opl/interface/stringutil.cpp)
   
#common commands for building c++ executables and libraries
add_library(tfd_module_api ${module_api_SOURCES})
add_library(tfd_opl_interface ${opl_interface_SOURCES})
add_executable(tfd_preprocess ${preprocess_SOURCES})
add_executable(tfd_search ${search_SOURCES})
add_dependencies(tfd_search continual_planning_msgs_generate_messages_cpp)
target_link_libraries(tfd_search ${catkin_LIBRARIES} tfd_module_api tfd_opl_interface dl)

#add_library(tfd_opl ${opl_SOURCES})
#add_executable(opl_translate_domain downward/opl/opl_translate_domain.cpp)
#target_link_libraries(opl_translate_domain tfd_opl dl)

set (CMAKE_CXX_FLAGS "-Wno-sign-compare -ansi -O3")
#add_compile_flags(tfd_preprocess "-Wno-sign-compare -ansi -pedantic")
#add_compile_flags(tfd_search "-Wno-sign-compare -ansi ")

#IF(${ROS_BUILD_TYPE} MATCHES Rel)
#   add_compile_flags(tfd_preprocess "-O3")
#   add_compile_flags(tfd_search "-O3")
#ENDIF(${ROS_BUILD_TYPE} MATCHES Rel)


find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

# needed: otherwise boost signals name clash
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
set(tfdm_interface_SOURCES
    interface/tfdm_interface.cpp
    interface/tfdm_eval_interface.cpp
    interface/domainParser.cpp
    interface/planParser.cpp
)
add_library(tfdm_interface ${tfdm_interface_SOURCES})
target_link_libraries(tfdm_interface ${catkin_LIBRARIES} ${QT_LIBRARIES})
