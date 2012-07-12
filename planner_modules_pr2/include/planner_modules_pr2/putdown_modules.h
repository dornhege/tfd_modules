#ifndef PUTDOWN_MODULES_H
#define PUTDOWN_MODULES_H

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include <map>
#include <string>
#include <utility>
#include <ros/ros.h>
#include <tidyup_msgs/GetPutdownPose.h>

using namespace modules;

/**
 * Simple module implementation for getting putdown poses from a service.
 */

static const bool g_Debug = false;

/// Retrieve a response with putdownPoses from the service.
/**
 * \returns true on success.
 */
bool getPutdownPoses(const ParameterList & parameterList, predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback, tidyup_msgs::GetPutdownPose::Response & putdownPoses);

/// Fill the GetPutdownPose Request from the planner state.
bool fillPutdownRequest(const ParameterList & parameterList, predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback, tidyup_msgs::GetPutdownPose::Request & request);


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

