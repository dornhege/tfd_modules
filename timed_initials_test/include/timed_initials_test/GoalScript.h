/*
 * testGoals.h
 *
 *  Created on: Aug 28, 2017
 *      Author: andreas
 */

#ifndef SRC_TESTGOALS_H_
#define SRC_TESTGOALS_H_

#include <continual_planning_executive/goalCreator.h>

class GoalScript : public continual_planning_executive::GoalCreator
{
public:
	GoalScript();
	virtual ~GoalScript();
	virtual void initialize(const std::deque<std::string> & arguments);
	virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
};

#endif /* SRC_TESTGOALS_H_ */
