#ifndef NAVSTACK_MODULE_H
#define NAVSTACK_MODULE_H

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include <map>
#include <string>
#include <utility>
#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>

using namespace modules;

/**
 * Simple module implementation for ROS navigation stack.
 *
 * Directly queries the move_base_node/make_plan service for each
 * cost request by the planner.
 */

static const bool g_Debug = false;

extern ros::NodeHandle* g_NodeHandle;
extern ros::ServiceClient g_GetPlan;

/// Plan requests are issued using this frame - so the poses from the planner are given in this frame (e.g. map)
extern std::string g_WorldFrame;

extern double g_GoalTolerance;

// Using a cache of queried path costs to prevent calling the path planning service multiple times
// Better: Can we assume symmetric path costs?
extern std::map< std::pair<std::string, std::string>, double> g_PathCostCache;

bool fillPathRequest(const ParameterList & parameterList, numericalFluentCallbackType numericalFluentCallback,
        nav_msgs::GetPlan::Request & request);

#ifdef __cplusplus
extern "C" {
#endif

void navstack_init(int argc, char** argv);

double pathCost(const ParameterList & parameterList, predicateCallbackType predicateCallback, 
        numericalFluentCallbackType numericalFluentCallback, int relaxed);

#ifdef __cplusplus
}
#endif

#endif

