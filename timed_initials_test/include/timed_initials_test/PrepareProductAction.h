/*
 * PrepareProductAction.h
 *
 *  Created on: Aug 29, 2017
 *      Author: andreas
 */

#ifndef SRC_PREPAREPRODUCTACTION_H_
#define SRC_PREPAREPRODUCTACTION_H_

#include <continual_planning_executive/actionExecutorInterface.h>

class PrepareProductAction : public continual_planning_executive::ActionExecutorInterface
{
public:
	PrepareProductAction();
	virtual ~PrepareProductAction();

	virtual void initialize(const std::deque<std::string> & arguments);
	virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const;
	virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState);
	virtual void cancelAction();
};

#endif /* SRC_PREPAREPRODUCTACTION_H_ */
