#ifndef NAVSTACK_MODULE_FULL_BODY_H
#define NAVSTACK_MODULE_FULL_BODY_H

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace modules;


extern ros::ServiceClient s_SwitchJointTopicClient;
extern ros::Subscriber s_JointStateSubscriber;
extern ros::Publisher s_PlanningJointStatePublisher;
extern sensor_msgs::JointState s_CurrentState;
extern sensor_msgs::JointState s_RightArmAtSide;
extern sensor_msgs::JointState s_LeftArmAtSide;

void jointStateCallback(const sensor_msgs::JointState& msg);
void replaceJointPosition(sensor_msgs::JointState& oldState, sensor_msgs::JointState& newJoints);
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

