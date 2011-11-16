#include "planExecutorInterface.h"
#include <iostream>

PlanExecutorInterface::PlanExecutorInterface()
{
    _onlyExecuteActionAtZeroTime = true;
}

PlanExecutorInterface::~PlanExecutorInterface()
{
}

bool PlanExecutorInterface::executeBlocking(const Plan & p)
{
    bool actionExectued = false;
    forEach(const DurativeAction & da, p.actions) {
        if(_onlyExecuteActionAtZeroTime && da.startTime > 0.0001)
            continue;

        bool count = 0;
        forEach(ActionExecutorInterface* ai, _actionExecutors) {
            if(ai->canExecute(da))
                count++;
        }
        if(count == 0) {
            std::cerr << "WARNING: No ActionExecutor for action: " << da << std::endl;
        }
        if(count > 1) {
            std::cerr << "WARNING: " << count << " ActionExecutors for action: " << da << std::endl;
        }
        forEach(ActionExecutorInterface* ai, _actionExecutors) {
            if(ai->canExecute(da)) {
                if(ai->executeBlocking(da))
                    actionExectued = true;
            }
        }
    }

    return actionExectued;
}

