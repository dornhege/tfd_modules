#include "continual_planning_executive/statusPublisher.h"
#include <sstream>
using std::stringstream;
using namespace continual_planning_msgs;

StatusPublisher::StatusPublisher()
{
    ros::NodeHandle nh;
    _pubStatus = nh.advertise<ContinualPlanningStatus>(
            "continual_planning_status", 10);
    _enabled = true;
}

StatusPublisher::~StatusPublisher()
{
}

void StatusPublisher::startedStateEstimation()
{
    publishStatus(ContinualPlanningStatus::STATE_ESTIMATION, ContinualPlanningStatus::ACTIVE, "");
}

void StatusPublisher::finishedStateEstimation(bool success, const SymbolicState & state, const SymbolicState & goal)
{
    SymbolicState::OStreamMode::forceNewlines = true;
    stringstream ss;
    ss << "Goal:" << std::endl;
    goal.toPDDLGoal(ss);
    ss << state;
    SymbolicState::OStreamMode::forceNewlines = false;
    publishStatus(ContinualPlanningStatus::STATE_ESTIMATION,
            success ? int(ContinualPlanningStatus::SUCCESS) : int(ContinualPlanningStatus::FAILURE), ss.str());
}


void StatusPublisher::startedMonitoring()
{
    publishStatus(ContinualPlanningStatus::MONITORING, ContinualPlanningStatus::ACTIVE, "");
}

void StatusPublisher::finishedMonitoring(bool success)
{
    publishStatus(ContinualPlanningStatus::MONITORING,
            success ? int(ContinualPlanningStatus::SUCCESS) : int(ContinualPlanningStatus::FAILURE), "");
}


void StatusPublisher::startedPlanning()
{
    publishStatus(ContinualPlanningStatus::PLANNING, ContinualPlanningStatus::ACTIVE, "");
}

void StatusPublisher::finishedPlanning(bool success, const Plan & plan)
{
    stringstream ss;
    ss << std::fixed << std::setprecision(2) << plan;
    publishStatus(ContinualPlanningStatus::PLANNING,
            success ? int(ContinualPlanningStatus::SUCCESS) : int(ContinualPlanningStatus::FAILURE), ss.str());
}


void StatusPublisher::startedExecution(const DurativeAction & a)
{
    stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << a;
    publishStatus(ContinualPlanningStatus::EXECUTION, ContinualPlanningStatus::ACTIVE, ss.str());
}

void StatusPublisher::finishedExecution(bool success, const DurativeAction & a)
{
    stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << a;
    publishStatus(ContinualPlanningStatus::EXECUTION,
            success ? int(ContinualPlanningStatus::SUCCESS) : int(ContinualPlanningStatus::FAILURE), ss.str());
}


void StatusPublisher::updateCurrentPlan(const Plan & plan)
{
    stringstream ss;
    ss << std::fixed << std::setprecision(2) << plan;
    publishStatus(ContinualPlanningStatus::CURRENT_PLAN, ContinualPlanningStatus::ACTIVE, ss.str());
}


void StatusPublisher::finishedContinualPlanning(bool success, const std::string & result)
{
    publishStatus(ContinualPlanningStatus::CONTINUAL_PLANNING_FINISHED,
            success ? int(ContinualPlanningStatus::SUCCESS) : int(ContinualPlanningStatus::FAILURE), result);
}

void StatusPublisher::publishStatus(int component, int status, const std::string & description)
{
    ContinualPlanningStatus msg;
    msg.component = component;
    msg.status = status;
    msg.description = description;

    if(_enabled && _pubStatus)
        _pubStatus.publish(msg);
}

