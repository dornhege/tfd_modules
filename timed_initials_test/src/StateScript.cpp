/*
 * StateScript.cpp
 *
 *  Created on: Aug 29, 2017
 *      Author: andreas
 */

#include <timed_initials_test/StateScript.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(StateScript, continual_planning_executive::StateCreator)

StateScript::StateScript()
{
}

StateScript::~StateScript()
{
}

void StateScript::initialize(const std::deque<std::string> & arguments)
{
}

bool StateScript::fillState(SymbolicState & state)
{
	return true;
}
