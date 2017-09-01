/*
 * AssembleProductAction.cpp
 *
 *  Created on: Aug 29, 2017
 *      Author: andreas
 */

#include <timed_initials_test/AssembleProductAction.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(AssembleProductAction, continual_planning_executive::ActionExecutorInterface)

AssembleProductAction::AssembleProductAction()
{
}

AssembleProductAction::~AssembleProductAction()
{
}

void AssembleProductAction::initialize(const std::deque<std::string> & arguments)
{
}

bool AssembleProductAction::canExecute(const DurativeAction & a, const SymbolicState & currentState) const
{
	return a.name == "assemble-product";
}

bool AssembleProductAction::executeBlocking(const DurativeAction & a, SymbolicState & currentState)
{
	currentState.setBooleanPredicate(Predicate("machine-idle", a.parameters[1]), false);
	ros::Duration execution_time(5);
	execution_time.sleep();
	currentState.setBooleanPredicate(Predicate("machine-idle", a.parameters[1]), true);
	currentState.setBooleanPredicate(Predicate("product-assembled", a.parameters[0]), true);
	return true;
}

void AssembleProductAction::cancelAction()
{
}
