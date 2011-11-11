#ifndef PLAN_EXECUTOR_INTERFACE_H
#define PLAN_EXECUTOR_INTERFACE_H

#include "symbolicState.h"
#include "plan.h"
#include "actionExecutorInterface.h"

class PlanExecutorInterface
{
    public:
        PlanExecutorInterface();
        ~PlanExecutorInterface();

        virtual void createActionExecutors() = 0;

        virtual bool executeBlocking(const Plan & p);

    protected:
        bool _onlyExecuteActionAtZeroTime;    ///< if false any action will be executed that an actionexecutor wants to

        deque<ActionExecutorInterface*> _actionExecutors;
};

#endif

