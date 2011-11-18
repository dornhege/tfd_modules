#include "planExecutor.h"
#include <ros/ros.h>
#include <iostream>

PlanExecutor::PlanExecutor()
{
    _onlyExecuteActionAtZeroTime = true;
}

PlanExecutor::~PlanExecutor()
{
}

void PlanExecutor::addActionExecutor(continual_planning_executive::ActionExecutorInterface* ae)
{
    _actionExecutors.push_back(ae);
}

bool PlanExecutor::executeBlocking(const Plan & p, SymbolicState & currentState)
{
    bool actionExectued = false;
    forEach(const DurativeAction & da, p.actions) {
        if(_onlyExecuteActionAtZeroTime && da.startTime > 0.0001)
            continue;

        bool count = 0;
        forEach(continual_planning_executive::ActionExecutorInterface* ai, _actionExecutors) {
            if(ai->canExecute(da, currentState))
                count++;
        }
        if(count == 0) {
            std::cerr << "WARNING: No ActionExecutor for action: " << da << std::endl;
        }
        if(count > 1) {
            std::cerr << "WARNING: " << count << " ActionExecutors for action: " << da << std::endl;
        }
        forEach(continual_planning_executive::ActionExecutorInterface* ai, _actionExecutors) {
            if(ai->canExecute(da, currentState)) {
                if(ai->executeBlocking(da, currentState)) {
                    actionExectued = true;
                    ROS_INFO_STREAM("Successfully executed action: \"" << da << "\"");
                } else {
                    ROS_WARN_STREAM("Action execution failed for action: \"" << da << "\"");
                }
            }
        }
    }

    return actionExectued;
}

