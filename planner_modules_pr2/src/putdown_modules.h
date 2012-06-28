#ifndef PUTDOWN_MODULES_H
#define PUTDOWN_MODULES_H

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include <map>
#include <string>
#include <utility>
#include <ros/ros.h>

using namespace modules;

/**
 * Simple module implementation for ROS navigation stack.
 *
 * Directly queries the move_base_node/make_plan service for each
 * cost request by the planner.
 */

static const bool g_Debug = false;

#ifdef __cplusplus
extern "C" {
#endif

void putdown_init(int argc, char** argv);

double canPutdown(const ParameterList & parameterList, predicateCallbackType predicateCallback, 
        numericalFluentCallbackType numericalFluentCallback, int relaxed);

int updatePutdownPose(const ParameterList & parameterList, predicateCallbackType predicateCallback, 
        numericalFluentCallbackType numericalFluentCallback, std::vector<double> & writtenVars);


#ifdef __cplusplus
}
#endif

#endif

