/*
 * PrepareProductAction.cpp
 *
 *  Created on: Aug 29, 2017
 *      Author: andreas
 */

#include <ros/ros.h>
#include <timed_initials_test/PrepareProductAction.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(PrepareProductAction, continual_planning_executive::ActionExecutorInterface)

PrepareProductAction::PrepareProductAction()
{
}

PrepareProductAction::~PrepareProductAction()
{
}

void PrepareProductAction::initialize(const std::deque<std::string> & arguments)
{
//	ROS_INFO_STREAM("prepare-product action loaded");
}

bool PrepareProductAction::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
{
	ROS_INFO_STREAM(a.name);
	return a.name == "prepare-product";
}

bool PrepareProductAction::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
{
	ros::Duration execution_time(5);
	execution_time.sleep();
	currentState.setBooleanPredicate(Predicate("product-prepared", a.parameters[0]), true);
	return true;
}

void PrepareProductAction::cancelAction()
{
}
