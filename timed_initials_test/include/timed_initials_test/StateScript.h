/*
 * StateScript.h
 *
 *  Created on: Aug 29, 2017
 *      Author: andreas
 */

#ifndef SRC_STATESCRIPT_H_
#define SRC_STATESCRIPT_H_

#include <continual_planning_executive/stateCreator.h>

class StateScript : public continual_planning_executive::StateCreator
{
public:
	StateScript();
	virtual ~StateScript();

	virtual void initialize(const std::deque<std::string> & arguments);
	virtual bool fillState(SymbolicState & state);
private:
};

#endif /* SRC_STATESCRIPT_H_ */
