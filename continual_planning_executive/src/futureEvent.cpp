/*
 * ExpectedFutureEffects.cpp
 *
 *  Created on: Aug 25, 2017
 *      Author: andreas
 */

#include <continual_planning_executive/futureEvent.h>

FutureEvent::FutureEvent(double time)
{
	setTriggerTime(time);
}

FutureEvent::~FutureEvent()
{
}

void FutureEvent::clear()
{
	object_fluents.clear();
	boolean_fluents.clear();
	numerical_fluents.clear();
}

void FutureEvent::setBooleanFluent(const Predicate& p, bool value)
{
	boolean_fluents[p] = value;
}

void FutureEvent::setObjectFluent(const Predicate& p, const string& value)
{
	object_fluents[p] = value;
}

void FutureEvent::setNumericalFluent(const Predicate& p, double value)
{
	numerical_fluents[p] = value;
}

bool FutureEvent::triggered() const
{
	return time < ros::Time::now();
}

double FutureEvent::getTriggerTime() const
{
	return (time - ros::Time::now()).toSec();
}

void FutureEvent::toPDDL(std::ostream & os) const
{
	size_t count = boolean_fluents.size()+numerical_fluents.size()+object_fluents.size();
	if (count == 0)
	{
		return;
	}
	string indent = "    ";
	bool break_lines = count > 1;
	os << indent << "(at "<<getTriggerTime()<<" ";
	if (count > 1)
	{
		os << "(and ";
	}
	forEach (const PredicateBooleanMap::value_type& fluent, boolean_fluents)
	{
		if(fluent.second)
		{
			if (break_lines)
			{
				os << indent << indent;
			}
			os << fluent.first;
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

const PredicateBooleanMap& FutureEvent::getBooleanFluents() const
{
	return boolean_fluents;
}

const PredicateDoubleMap& FutureEvent::getNumericalFluents() const
{
	return numerical_fluents;
}

const PredicateStringMap& FutureEvent::getObjectFluents() const
{
	return object_fluents;
}

