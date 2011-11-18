#include "continualPlanning.h"
#include <ros/ros.h>

ContinualPlanning::ContinualPlanning() : _planner(NULL)
{
}

ContinualPlanning::~ContinualPlanning()
{
}

bool ContinualPlanning::loop()
{
    if(!estimateCurrentState()) {
        ROS_WARN("State estimation failed.");     // FIXME: continue execution until this works?
        return true;
    }

    if(isGoalFulfilled()) // done!
        return false;

    Plan newPlan = monitorAndReplan();
    if(newPlan.empty()) {
        ROS_ERROR("Empty plan returned by monitorAndReplan.");
        return false;           // can't do anything
    }

    // exec plan
    _currentPlan = newPlan;

    // TODO: just send and remember actions
    // supervise those while running and estimating state.
    if(!_planExecutor.executeBlocking(_currentPlan, _currentState)) {
        ROS_WARN_STREAM("No action was executed for current plan:\n" << _currentPlan);
    }

    return true;
}

bool ContinualPlanning::isGoalFulfilled() const
{
    return _goal.isFulfilledBy(_currentState);
}

bool ContinualPlanning::estimateCurrentState()
{
    bool ret = true;
    for(std::vector<continual_planning_executive::StateCreator*>::iterator it = _stateCreators.begin();
            it != _stateCreators.end(); it++) {
        ret &= (*it)->fillState(_currentState);
    }
    ROS_INFO_STREAM("Current state is: " << _currentState);
    return ret;
}

Plan ContinualPlanning::monitorAndReplan() const
{
    if(!needReplanning()) {
        return _currentPlan;
    }

    // fancy prefix stuff ???

    // REPLAN
    Plan plan;
    continual_planning_executive::PlannerInterface::PlannerResult result = 
        _planner->plan(_currentState, _goal, plan);

    if(result == continual_planning_executive::PlannerInterface::PR_SUCCESS
            || result == continual_planning_executive::PlannerInterface::PR_SUCCESS_TIMEOUT) {
        ROS_INFO("Planning successfull.");
    } else {
        ROS_WARN("Planning failed, result: %s",
                continual_planning_executive::PlannerInterface::PlannerResultStr(result).c_str());
    }
    return plan;
}

bool ContinualPlanning::needReplanning() const
{
    static SymbolicState lastReplanState;

    if(_currentPlan.empty()) {
        lastReplanState = _currentState;
        return true;
    }

    // TODO call monitoring and check for assertions

    // TODO parametrize replanning method and state comp method
    if(_currentState.booleanEquals(lastReplanState))
        return false;

    ROS_INFO("state changed since last replanning.");
    lastReplanState = _currentState;

    // later: app(current, currentPlan) != goal ==== monitoring

    // yeah...
    return true;
}

