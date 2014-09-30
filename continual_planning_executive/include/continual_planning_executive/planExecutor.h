#ifndef PLAN_EXECUTOR_H
#define PLAN_EXECUTOR_H

#include <set>
#include <fstream>
#include "continual_planning_executive/symbolicState.h"
#include "continual_planning_executive/plan.h"
#include "continual_planning_executive/actionExecutorInterface.h"

class PlanExecutor
{
    public:
        PlanExecutor();
        ~PlanExecutor();

        void addActionExecutor(boost::shared_ptr<continual_planning_executive::ActionExecutorInterface> ae);

        virtual bool executeBlocking(const Plan & p, SymbolicState & currentState,
                std::set<DurativeAction> & executedActions);

        /// send a cancel signal to all actions.
        void cancelAllActions();

    protected:
        void checkActionTimesFile();

    protected:
        bool _onlyExecuteActionAtZeroTime;    ///< if false any action will be executed that an actionexecutor wants to

        deque<boost::shared_ptr<continual_planning_executive::ActionExecutorInterface> > _actionExecutors;

        bool _recordActionTimes;
        std::ofstream _actionTimesFile;
};

#endif

