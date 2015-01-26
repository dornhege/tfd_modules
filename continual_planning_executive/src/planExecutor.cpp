#include "continual_planning_executive/planExecutor.h"
#include <ros/ros.h>
#include <iostream>

PlanExecutor::PlanExecutor()
{
    _onlyExecuteActionAtZeroTime = true;
    _recordActionTimes = true;
}

PlanExecutor::~PlanExecutor()
{
    if(_actionTimesFile.good())
        _actionTimesFile.close();
}

void PlanExecutor::checkActionTimesFile()
{
    if(!_recordActionTimes)
        return;

    if(_actionTimesFile.good() && _actionTimesFile.is_open())
        return;

    ros::NodeHandle nhConfig("/tfd_modules/eval");
    string evalCurDir;
    if(!nhConfig.getParam("eval_current_dir", evalCurDir)) {
        ROS_ERROR("PlanExecutor: /tfd_modules/eval/eval_current_dir was not set!");
        evalCurDir = ".";
    }
    string actionTimesFileName = evalCurDir + "/action.times";

    _actionTimesFile.open(actionTimesFileName.c_str());
    if(!_actionTimesFile.good()) {
        ROS_ERROR("Failed to open action times file at: %s", actionTimesFileName.c_str());
    }
}

void PlanExecutor::addActionExecutor(boost::shared_ptr<continual_planning_executive::ActionExecutorInterface> ae)
{
    _actionExecutors.push_back(ae);
}

bool PlanExecutor::executeBlocking(const Plan & p, SymbolicState & currentState,
                std::set<DurativeAction> & executedActions)
{
    checkActionTimesFile();

    int actionsExectued = 0;
    forEach(const DurativeAction & da, p.actions) {
        if(_onlyExecuteActionAtZeroTime && da.startTime > 0.01)
            continue;

        bool count = 0;
        forEach(boost::shared_ptr<continual_planning_executive::ActionExecutorInterface> ai, _actionExecutors) {
            if(ai->canExecute(da, currentState))
                count++;
        }
        if(count == 0) {
            std::cerr << "WARNING: No ActionExecutor for action: " << da << std::endl;
        }
        if(count > 1) {
            std::cerr << "WARNING: " << count << " ActionExecutors for action: " << da << std::endl;
        }
        forEach(boost::shared_ptr<continual_planning_executive::ActionExecutorInterface> ai, _actionExecutors) {
            if(ai->canExecute(da, currentState)) {
                ROS_INFO_STREAM("Trying to execute action: \"" << da << "\"");

                ros::WallTime startTime = ros::WallTime::now();
                if(ai->executeBlocking(da, currentState)) {
                    actionsExectued++;
                    ROS_INFO_STREAM("Successfully executed action: \"" << da << "\"");

                    if(_actionTimesFile.good()) {
                        ros::WallTime endTime = ros::WallTime::now();
                        _actionTimesFile << "\"" << da << "\" 1 " << (endTime - startTime).toSec() << std::endl;
                        _actionTimesFile.flush();
                    }
                } else {
                    ROS_ERROR_STREAM("Action execution failed for action: \"" << da << "\"");

                    if(_actionTimesFile.good()) {
                        ros::WallTime endTime = ros::WallTime::now();
                        _actionTimesFile << "\"" << da << "\" 0 " << (endTime - startTime).toSec() << std::endl;
                        _actionTimesFile.flush();
                    }
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

void PlanExecutor::cancelAllActions()
{
    forEach(boost::shared_ptr<continual_planning_executive::ActionExecutorInterface> ai, _actionExecutors) {
        ai->cancelAction();
    }
}

