/*
 * DeliverProductAction.h
 *
 *  Created on: Aug 29, 2017
 *      Author: andreas
 */

#ifndef SRC_DELIVERPRODUCTACTION_H_
#define SRC_DELIVERPRODUCTACTION_H_

#include <continual_planning_executive/actionExecutorInterface.h>

class DeliverProductAction : public continual_planning_executive::ActionExecutorInterface
{
public:
	DeliverProductAction();
	virtual ~DeliverProductAction();

	virtual void initialize(const std::deque<std::string> & arguments);
	virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const;
	virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState);
	virtual void cancelAction();
};

#endif /* SRC_DELIVERPRODUCTACTION_H_ */
