/*
 * ExpectedFutureEffects.cpp
 *
 *  Created on: Aug 25, 2017
 *      Author: andreas
 */

#include <continual_planning_executive/expectedFutureEvent.h>

ExpectedFutureEvent::ExpectedFutureEvent(double time): time(time)
{

}

ExpectedFutureEvent::~ExpectedFutureEvent()
{
}

void ExpectedFutureEvent::clear()
{
	object_fluents.clear();
	boolean_fluents.clear();
	numerical_fluents.clear();
}

void ExpectedFutureEvent::setBooleanFluent(const Predicate& p, bool value)
{
	boolean_fluents[p] = value;
}

void ExpectedFutureEvent::setObjectFluent(const Predicate& p, const string& value)
{
	object_fluents[p] = value;
}

void ExpectedFutureEvent::setNumericalFluent(const Predicate& p, double value)
{
	numerical_fluents[p] = value;
}

void ExpectedFutureEvent::setTime(double time)
{
	this->time = time;
}

double ExpectedFutureEvent::getTime() const
{
	return time;
}

