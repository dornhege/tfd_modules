/*
 * testGoals.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: andreas
 */

#include <continual_planning_executive/futureEvent.h>
#include <timed_initials_test/GoalScript.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(GoalScript, continual_planning_executive::GoalCreator)

GoalScript::GoalScript()
{
}

GoalScript::~GoalScript()
{
}

void GoalScript::initialize(const std::deque<std::string> & arguments)
{

}

bool GoalScript::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
{
	currentState.addSuperType("product", "object");
	currentState.addSuperType("machine", "object");
	currentState.addObject("p1", "product");
	currentState.addObject("m1", "machine");

	FutureEvent::Ptr machine_idle_m1 = boost::make_shared<FutureEvent>(20);
	machine_idle_m1->setBooleanFluent(Predicate("machine-idle", "m1"), true);
	currentState.addFutureEvent(machine_idle_m1);
	FutureEvent::Ptr delivery_possible_p1 = boost::make_shared<FutureEvent>(60);
	delivery_possible_p1->setBooleanFluent(Predicate("delivery-possible", "p1"), true);
	currentState.addFutureEvent(delivery_possible_p1);

	goal = currentState;
	goal.setBooleanPredicate(Predicate("product-delivered", "p1"), true);

	return true;
}
