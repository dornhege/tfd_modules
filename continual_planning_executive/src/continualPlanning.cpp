#include "continual_planning_executive/continualPlanning.h"
#include <ros/ros.h>
#include <iomanip>
#include <iostream>

ContinualPlanning::ContinualPlanning()
{
    _replanningTrigger = ReplanByMonitoring;
    _allowDirectGoalCheck = false;      // OK, we do this via monitoring
    _forceReplan = true;
    _initialStateEstimated = false;
}

ContinualPlanning::~ContinualPlanning()
{
}

void ContinualPlanning::addGoalCreator(boost::shared_ptr<continual_planning_executive::GoalCreator> gc)
{
    _goalCreators.push_back(gc);
}

void ContinualPlanning::addStateCreator(boost::shared_ptr<continual_planning_executive::StateCreator> sc)
{
    _stateCreators.push_back(sc);
}

void ContinualPlanning::addActionExecutor(boost::shared_ptr<continual_planning_executive::ActionExecutorInterface> ae)
{
    _planExecutor.addActionExecutor(ae);
}

void ContinualPlanning::setPlanner(boost::shared_ptr<continual_planning_executive::PlannerInterface> pi)
{
    _planner = pi;
}

void ContinualPlanning::reset()
{
    _planExecutor.cancelAllActions();
    _initialStateEstimated = false;
    _currentState = SymbolicState();
    _goal = SymbolicState();
}

ContinualPlanning::ContinualPlanningState ContinualPlanning::loop()
{
    if (! _initialStateEstimated)
    {
        if (! estimateInitialStateAndGoal())
        {
            ROS_WARN("Initial state estimation failed.");
            return Running;
        }
    }
    if(!estimateCurrentState()) {
        ROS_WARN("State estimation failed.");     // FIXME: continue execution until this works.
        return Running;
    }

    if(_allowDirectGoalCheck && isGoalFulfilled()) { // done!
        _status.finishedContinualPlanning(true, "Goal fulfilled");
        return FinishedAtGoal;
    }

    bool atGoal = false;
    Plan newPlan = monitorAndReplan(atGoal);
    if(newPlan.empty()) {
        if(atGoal) {
            ROS_INFO("\n\nEmpty plan returned by monitorAndReplan - Reached Goal!\n\n");
            _status.finishedContinualPlanning(true, "Empty plan returned by monitorAndReplan - Reached Goal!");
            return FinishedAtGoal;
        }

        ROS_ERROR("\n\nEmpty plan returned by monitorAndReplan - not at goal.\n\n");
        _status.finishedContinualPlanning(false, "Empty plan returned by monitorAndReplan - not at goal.");
        return FinishedNoPlanToGoal;           // not at goal and no plan -> can't do anything besides fail
    }

    // exec plan
    _currentPlan = newPlan;
    _status.updateCurrentPlan(_currentPlan);

    // TODO_TP: just send and remember actions
    //          supervise those while running and estimating state.
    std::set<DurativeAction> executedActions;
    // should not be empty (see above), FIXME exec should only exec the first
    _status.startedExecution(_currentPlan.actions.front()); 
    if(!_planExecutor.executeBlocking(_currentPlan, _currentState, executedActions)) {
        _status.finishedExecution(false, _currentPlan.actions.front());
        ROS_ERROR_STREAM("No action was executed for current plan:\n" << _currentPlan << "\nWaiting for 10 sec...");
        _forceReplan = true;        // force here in the hope that it fixes something.
        ros::WallDuration sleep(10.0);
        sleep.sleep();
    } else {
        _status.finishedExecution(true, _currentPlan.actions.front());
    }

    // remove executedActions from plan
    for(std::set<DurativeAction>::iterator it = executedActions.begin(); it != executedActions.end(); it++) {
        _currentPlan.removeAction(*it);
    }
    _status.updateCurrentPlan(_currentPlan);

    return Running;
}

bool ContinualPlanning::executeActionDirectly(const DurativeAction & a, bool publishStatus)
{
    _status.setEnabled(publishStatus);

    if(!estimateCurrentState()) {
        ROS_ERROR("State estimation failed.");
        _status.setEnabled(true);
        return false;
    }

    Plan plan;
    plan.actions.push_back(a);

    std::set<DurativeAction> executedActions;
    // should not be empty (see above), FIXME exec should only exec the first
    _status.startedExecution(plan.actions.front()); 
    if(!_planExecutor.executeBlocking(plan, _currentState, executedActions)) {
        _status.finishedExecution(false, plan.actions.front());
        ROS_ERROR_STREAM("No action was executed for current plan:\n" << plan);
        _status.setEnabled(true);
        return false;
    } else {
        _status.finishedExecution(true, plan.actions.front());
    }

    _status.setEnabled(true);
    return !executedActions.empty();
}

bool ContinualPlanning::isGoalFulfilled() const
{
    return _goal.isFulfilledBy(_currentState);
}

bool ContinualPlanning::estimateInitialStateAndGoal()
{
    ROS_INFO("Estimating initial state and setting goal condition.");
    _initialStateEstimated = true;
    forEach(boost::shared_ptr<continual_planning_executive::GoalCreator> gc, _goalCreators)
    {
        _initialStateEstimated &= gc->fillStateAndGoal(_currentState, _goal);
    }
    return _initialStateEstimated;
}

bool ContinualPlanning::estimateCurrentState()
{
    _status.startedStateEstimation();
    bool ret = true;
    forEach(boost::shared_ptr<continual_planning_executive::StateCreator> sc, _stateCreators)
    {
        ret &= sc->fillState(_currentState);
    }
    ROS_INFO_STREAM("Current state is: " << _currentState << std::endl);
    _status.finishedStateEstimation(ret, _currentState, _goal);
    return ret;
}

Plan ContinualPlanning::monitorAndReplan(bool & atGoal)
{
    _status.startedMonitoring();
    if(!needReplanning(atGoal)) {
        _status.finishedMonitoring(true);
        _status.publishStatus(continual_planning_msgs::ContinualPlanningStatus::PLANNING,
                continual_planning_msgs::ContinualPlanningStatus::INACTIVE, "-");
        return _currentPlan;
    }
    _status.finishedMonitoring(false);  // need replanning -> monitoring false

    // REPLAN
    Plan plan;
    _status.startedPlanning();
    continual_planning_executive::PlannerInterface::PlannerResult result = 
        _planner->plan(_currentState, _goal, plan);
    _forceReplan = false;       // did replanning

    if(result == continual_planning_executive::PlannerInterface::PR_SUCCESS
            || result == continual_planning_executive::PlannerInterface::PR_SUCCESS_TIMEOUT) {
        ROS_INFO_STREAM("Planning successfull. Got plan:" << std::endl
                << std::fixed << std::setprecision(2) << plan);
        _status.finishedPlanning(true, plan);
    } else {
        ROS_ERROR("Planning failed, result: %s",
                continual_planning_executive::PlannerInterface::PlannerResultStr(result).c_str());
        _status.finishedPlanning(false, plan);
    }
    return plan;
}

bool ContinualPlanning::needReplanning(bool & atGoal) const
{
    static SymbolicState lastReplanState;

    if(_forceReplan) {
        ROS_WARN("Replanning was forced.");
        return true;
    }

    // in monitoring it is OK to monitor the empty plan - only if that fails we need to replan
    if(_currentPlan.empty() && _replanningTrigger != ReplanByMonitoring) {
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
                continual_planning_executive::PlannerInterface::PlannerResult result = 
                    _planner->monitor(_currentState, _goal, _currentPlan);
                if(result == continual_planning_executive::PlannerInterface::PR_SUCCESS) {
                    if(_currentPlan.empty())
                        atGoal = true;
                    return false;   // success = _currentPlan leads to goal -> no replanning
                } else if(result == continual_planning_executive::PlannerInterface::PR_FAILURE_UNREACHABLE) {
                    return true;
                } else {
                    ROS_WARN("Unexpected monitoring result: %s",
                            continual_planning_executive::PlannerInterface::PlannerResultStr(result).c_str());
                    // there should be no timeouts in monitoring
                    if(result == continual_planning_executive::PlannerInterface::PR_SUCCESS_TIMEOUT) {
                        if(_currentPlan.empty())
                            atGoal = true;
                        return false;
                    }
                    return true;
                }
            }
            break;
    }

    return true;
}

