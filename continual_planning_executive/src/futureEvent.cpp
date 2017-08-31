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
	string indent = "    ";
	bool break_lines = count > 1;
	os << indent << "(at "<<time<<" ";
	if (count > 1)
	{
		os << "(and ";
	}
	forEach (const PredicateBooleanMap::value_type& fluent, boolean_fluents)
	{
		if(fluent.second)
		{
			os << indent << indent << "("<<fluent.first << ")";
			if (break_lines)
			{
				os << std::endl;
			}
		}
	}
	forEach (const PredicateDoubleMap::value_type& fluent, numerical_fluents)
	{
		os << indent << indent << "(= "<<fluent.first << " " << fluent.second << ")";
		if (break_lines)
		{
			os << std::endl;
		}
	}
	forEach (const PredicateStringMap::value_type& fluent, object_fluents)
	{
		os << indent << indent << "(= "<<fluent.first << " " << fluent.second << ")";
		if (break_lines)
		{
			os << std::endl;
		}
	}
	if (count > 1)
	{
		os << indent << ")";
	}
	os << ")"<<std::endl;
}
