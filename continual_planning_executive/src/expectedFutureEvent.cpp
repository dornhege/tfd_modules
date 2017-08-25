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

void ExpectedFutureEvent::toPDDL(std::ostream & os) const
{
	size_t count = boolean_fluents.size()+numerical_fluents.size()+object_fluents.size();
	if (count == 0)
	{
		return;
	}

	os << "    (at "<<time<<" ";
	if (count > 1)
	{
		os << "(and ";
	}
	formatFluents(os, "        ", string(std::endl));
	os << ")"<<std::endl;
}

void ExpectedFutureEvent::formatFluents(std::ostream & os, const string& indent, const string& linebreak) const
{
	forEach (const PredicateBooleanMap::value_type& fluent, boolean_fluents)
	{
		if(fluent.second)
		{
			os << indent << "("<<fluent.first << ")" << linebreak;
		}
	}
	forEach (const PredicateDoubleMap::value_type& fluent, numerical_fluents)
	{
		os << indent << "(= "<<fluent.first << " " << fluent.second << ")" << linebreak;
	}
	forEach (const PredicateStringMap::value_type& fluent, object_fluents)
	{
		os << indent << "(= "<<fluent.first << " " << fluent.second << ")" << linebreak;
	}
}
