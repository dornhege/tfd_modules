#ifndef NAVSTACK_MODULE_FULL_BODY_H
#define NAVSTACK_MODULE_FULL_BODY_H

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace modules;

void jointStateCallback(const sensor_msgs::JointState& msg);
void publishPlanningArmState();
void switchToExecutionTopic();

#ifdef __cplusplus
extern "C" {
#endif

void fullbody_navstack_init(int argc, char** argv);

double fullbody_pathCost(const ParameterList & parameterList, predicateCallbackType predicateCallback,
      numericalFluentCallbackType numericalFluentCallback, int relaxed);

#ifdef __cplusplus
}
#endif


#endif

