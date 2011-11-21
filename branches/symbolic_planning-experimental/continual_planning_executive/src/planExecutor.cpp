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

bool PlanExecutor::executeBlocking(const Plan & p, SymbolicState & currentState,
                std::set<DurativeAction> & executedActions)
{
    int actionsExectued = 0;
    forEach(const DurativeAction & da, p.actions) {
        if(_onlyExecuteActionAtZeroTime && da.startTime > 0.01)
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
                ROS_INFO_STREAM("Trying to execute action: \"" << da << "\"");
                if(ai->executeBlocking(da, currentState)) {
                    actionsExectued++;
                    ROS_INFO_STREAM("Successfully executed action: \"" << da << "\"");
                } else {
                    ROS_WARN_STREAM("Action execution failed for action: \"" << da << "\"");
                }
                // FIXME: insert even if failed as we tried and the action is "used up"
                executedActions.insert(da);
            }
        }
    }

    if(actionsExectued > 1)
        ROS_WARN("Executed %d actions in one step.", actionsExectued);

    return actionsExectued > 0;
}

