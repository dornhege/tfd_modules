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

#include <boost/shared_ptr.hpp>
#include <continual_planning_executive/predicate.h>
#include <map>

using std::map;
using std::string;

class ExpectedFutureEvent
{
public:
	typedef boost::shared_ptr<ExpectedFutureEvent> Ptr;
	typedef boost::shared_ptr<ExpectedFutureEvent const> ConstPtr;
	ExpectedFutureEvent(double time);
	virtual ~ExpectedFutureEvent();

	void clear();
	void setBooleanFluent(const Predicate& p, bool value);
	void setObjectFluent(const Predicate& p, const string& value);
	void setNumericalFluent(const Predicate& p, double value);
	/**
	 * @param time in seconds from now
	 */
	void setTime(double time);
	double getTime() const;

	void toPDDL(std::ostream & os) const;

private:
	void formatFluents(std::ostream & os, const string& indent, bool break_lines) const;
	double time;
	PredicateBooleanMap boolean_fluents;
	PredicateDoubleMap numerical_fluents;
	PredicateStringMap object_fluents;
};

#endif /* SRC_EXPECTEDFUTUREEFFECTS_H_ */
