#ifndef PLAN_EXECUTOR_INTERFACE_H
#define PLAN_EXECUTOR_INTERFACE_H

#include "continual_planning_executive/symbolicState.h"
#include "continual_planning_executive/plan.h"
#include "continual_planning_executive/actionExecutorInterface.h"

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

