/*
 * ExpectedFutureEffects.h
 *
 *  Created on: Aug 25, 2017
 *      Author: andreas
 */

#ifndef SRC_EXPECTEDFUTUREEFFECTS_H_
#define SRC_EXPECTEDFUTUREEFFECTS_H_
#include <boost/foreach.hpp>
#ifdef __CDT_PARSER__
#define forEach(a, b) for(a : b)
#else
#define forEach BOOST_FOREACH
#endif

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <continual_planning_executive/predicate.h>
#include <map>

using std::map;
using std::string;

class FutureEvent
{
public:
	typedef boost::shared_ptr<FutureEvent> Ptr;
	typedef boost::shared_ptr<FutureEvent const> ConstPtr;
	FutureEvent(double time);
	virtual ~FutureEvent();

	void clear();
	void setBooleanFluent(const Predicate& p, bool value);
	void setObjectFluent(const Predicate& p, const string& value);
	void setNumericalFluent(const Predicate& p, double value);
	/**
	 * @param time in seconds from now
	 */
	void setTriggerTime(double time)
	{
		this->time = ros::Time::now() + ros::Duration(time);
	}
	double getTriggerTime() const;
	bool triggered() const;
	
	void toPDDL(std::ostream & os) const;
	const PredicateBooleanMap& getBooleanFluents() const;
	const PredicateDoubleMap& getNumericalFluents() const;
	const PredicateStringMap& getObjectFluents() const;

private:
	ros::Time time;
	PredicateBooleanMap boolean_fluents;
	PredicateDoubleMap numerical_fluents;
	PredicateStringMap object_fluents;
};

#endif /* SRC_EXPECTEDFUTUREEFFECTS_H_ */
