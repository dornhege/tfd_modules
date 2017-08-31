/*
 * DeliverProductAction.cpp
 *
 *  Created on: Aug 29, 2017
 *      Author: andreas
 */

#include <timed_initials_test/DeliverProductAction.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(DeliverProductAction, continual_planning_executive::ActionExecutorInterface)

DeliverProductAction::DeliverProductAction()
{
}

DeliverProductAction::~DeliverProductAction()
{
}

void DeliverProductAction::initialize(const std::deque<std::string> & arguments)
{
}

bool DeliverProductAction::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
{
	return a.name == "deliver-product";
}

bool DeliverProductAction::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
{
	ros::Duration execution_time(5);
	execution_time.sleep();
	currentState.setBooleanPredicate(Predicate("product-delivered", a.parameters[0]), true);
	return true;
}

void DeliverProductAction::cancelAction()
{
}
