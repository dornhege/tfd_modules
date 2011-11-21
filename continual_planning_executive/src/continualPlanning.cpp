#include "continualPlanning.h"
#include <ros/ros.h>

ContinualPlanning::ContinualPlanning() : _planner(NULL)
{
    _replanningTrigger = ReplanByMonitoring;
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
        return false;           // not at goal and no plan -> can't do anything besides fail
    }

    // exec plan
    _currentPlan = newPlan;

    // TODO_TP: just send and remember actions
    //          supervise those while running and estimating state.
    std::set<DurativeAction> executedActions;
    if(!_planExecutor.executeBlocking(_currentPlan, _currentState, executedActions)) {
        ROS_WARN_STREAM("No action was executed for current plan:\n" << _currentPlan);
    }
    // remove executedActions from plan
    for(std::set<DurativeAction>::iterator it = executedActions.begin(); it != executedActions.end(); it++) {
        _currentPlan.removeAction(*it);
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

    switch(_replanningTrigger) {
        case ReplanAlways:
            return true;
            break;
        case ReplanIfLogicalStateChanged:
            if(_currentState.booleanEquals(lastReplanState))
                return false;

            ROS_INFO("state changed since last replanning.");
            lastReplanState = _currentState;
            return true;
            break;
        case ReplanByMonitoring:
            // check that: app(currentState, currentPlan) reaches goal
            {
                // TODO remove executed actions from plan somewhere (is there a TODO for that)
                continual_planning_executive::PlannerInterface::PlannerResult result = 
                    _planner->monitor(_currentState, _goal, _currentPlan);
                if(result == continual_planning_executive::PlannerInterface::PR_SUCCESS) {
                    return false;   // success = _currentPlan leads to goal -> no replanning
                } else if(result == continual_planning_executive::PlannerInterface::PR_FAILURE_UNREACHABLE) {
                    return true;
                } else {
                    ROS_WARN("Unexpected monitoring result: %s",
                            continual_planning_executive::PlannerInterface::PlannerResultStr(result).c_str());
                    // there should be no timeouts in monitoring
                    if(result == continual_planning_executive::PlannerInterface::PR_SUCCESS_TIMEOUT)
                        return false;
                    return true;
                }
            }
            break;
    }

    return true;
}

