#ifndef PLAN_EXECUTOR_H
#define PLAN_EXECUTOR_H

#include "continual_planning_executive/symbolicState.h"
#include "continual_planning_executive/plan.h"
#include "continual_planning_executive/actionExecutorInterface.h"

class PlanExecutor
{
    public:
        PlanExecutor();
        ~PlanExecutor();

        void addActionExecutor(continual_planning_executive::ActionExecutorInterface* ae);

        virtual bool executeBlocking(const Plan & p, SymbolicState & currentState);

    protected:
        bool _onlyExecuteActionAtZeroTime;    ///< if false any action will be executed that an actionexecutor wants to

        deque<continual_planning_executive::ActionExecutorInterface*> _actionExecutors;
};

#endif

