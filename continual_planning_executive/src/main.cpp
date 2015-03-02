#include <stdio.h>
#include <ros/ros.h>
#include <vector>
#include <deque>
#include <string>
#include <sstream>
#include <signal.h>

#include "continual_planning_executive/symbolicState.h"
#include "continual_planning_executive/stateCreator.h"
#include "continual_planning_executive/goalCreator.h"
#include "continual_planning_executive/plannerInterface.h"
#include "continual_planning_msgs/SetContinualPlanningControl.h"
#include "continual_planning_msgs/ExecuteActionDirectly.h"
#include "continual_planning_executive/load_plugins.h"
#include "continual_planning_executive/planExecutor.h"
#include "continual_planning_executive/continualPlanning.h"

#include <QString>
#include <ros/package.h>

static ContinualPlanning* s_ContinualPlanning = NULL;

static int s_ContinualPlanningMode = continual_planning_msgs::SetContinualPlanningControl::Request::RUN;

bool setControlHandler(continual_planning_msgs::SetContinualPlanningControl::Request & req,
        continual_planning_msgs::SetContinualPlanningControl::Response & resp)
{
    switch(req.command) {
        case continual_planning_msgs::SetContinualPlanningControl::Request::RUN:
        case continual_planning_msgs::SetContinualPlanningControl::Request::PAUSE:
        case continual_planning_msgs::SetContinualPlanningControl::Request::STEP:
            if(s_ContinualPlanningMode != req.command) {
                ROS_INFO("Setting ContinualPlanningMode to %d", req.command);
            }
            s_ContinualPlanningMode = req.command;
            resp.command = s_ContinualPlanningMode;
            break;
        case continual_planning_msgs::SetContinualPlanningControl::Request::FORCE_REPLANNING:
            s_ContinualPlanning->forceReplanning();
            resp.command = req.command;
            break;
        case continual_planning_msgs::SetContinualPlanningControl::Request::REESTIMATE_STATE:
            if(!s_ContinualPlanning->estimateCurrentState()) {
                ROS_WARN("SetContinualPlanningControl: State estimation failed.");
                return false;
            }
            resp.command = req.command;
            break;
        default:
            ROS_ERROR("Invalid command in continual_planning_msgs::SetContinualPlanningControl: %d",
                    req.command);
            return false;
    }
    return true;
}

bool executeActionDirectlyHandler(continual_planning_msgs::ExecuteActionDirectly::Request & req,
        continual_planning_msgs::ExecuteActionDirectly::Response & resp)
{
    if(s_ContinualPlanningMode == continual_planning_msgs::SetContinualPlanningControl::Request::RUN) {
        ROS_WARN("Recevied executeActionDirectly request during RUN - ignoring, PAUSE first.");
        return false;
    }

    DurativeAction a(req.action);

    return s_ContinualPlanning->executeActionDirectly(a, true);
}

void signal_handler(int signal)
{
    if(signal != SIGINT) {
        raise(signal);
        return;
    }

    ROS_INFO("SIGINT received - canceling all running actions.");
    s_ContinualPlanning->_planExecutor.cancelAllActions();

    ROS_INFO("shutting down...");
    ros::shutdown();
}

/// Parse options for this node:
/**
 * If an action is given only this action is executed for debugging, e.g.:
 * drive-base robot_location door_kitchen_room1
 * arm-to-side left_arm
 *
 * \returns true, if the DurativeAction was filled and thus we should execute this action
 * instead of full continual planning
 */
bool parseOptions(int argc, char** argv, DurativeAction & a)
{
    if(argc < 2)
        return false;

    a.name = argv[1];
    for(int i = 2; i < argc; i++)
        a.parameters.push_back(argv[i]);
    return true;
}

int main(int argc, char** argv)
{
    ROS_INFO("Continual Planning Executive started.");

    unsigned int initOps = ros::init_options::NoSigintHandler;
    if(argc > 1)        // this is usually not the "main" continual_planning_executive, but a debug node
        initOps |= ros::init_options::AnonymousName;
    ros::init(argc, argv, "continual_planning_executive", initOps);
    signal(SIGINT, signal_handler);

    ros::NodeHandle nh;

    DurativeAction debugAction;
    bool executeDebug = parseOptions(argc, argv, debugAction);

    // clear module param cache
    if(!executeDebug) {
        nh.deleteParam("tfd_modules/module_cache");
    }

    s_ContinualPlanning = new ContinualPlanning();

    if(! load_plugins(s_ContinualPlanning)) {
        ROS_FATAL("Init failed.");
        return 1;
    }

    if(! s_ContinualPlanning->estimateInitialStateAndGoal()) {
        ROS_FATAL("Initial state failed.");
        return 1;
    }

    if(executeDebug) {
        bool execOK = s_ContinualPlanning->executeActionDirectly(debugAction, false);
        return execOK ? 0 : 1;
    }

    ros::ServiceServer serviceContinualPlanningMode =
        nh.advertiseService("set_continual_planning_control", setControlHandler);
    ros::ServiceServer serviceExecuteActionDirectly =
        nh.advertiseService("execute_action_directly", executeActionDirectlyHandler);

    ros::NodeHandle nhPriv("~");
    bool paused = false;
    nhPriv.param("start_paused", paused, false);
    if(paused)
        s_ContinualPlanningMode = continual_planning_msgs::SetContinualPlanningControl::Request::PAUSE;

    ros::Rate loopSleep(5);
    ContinualPlanning::ContinualPlanningState cpState = ContinualPlanning::Running;
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    while(ros::ok()) {
//        ros::spinOnce();

        if(s_ContinualPlanningMode == continual_planning_msgs::SetContinualPlanningControl::Request::RUN
            || s_ContinualPlanningMode == continual_planning_msgs::SetContinualPlanningControl::Request::STEP) {
            cpState = s_ContinualPlanning->loop();
            if(cpState != ContinualPlanning::Running) {
                break;
            }
            // STEP means RUN once.
            if(s_ContinualPlanningMode == continual_planning_msgs::SetContinualPlanningControl::Request::STEP)
                s_ContinualPlanningMode = continual_planning_msgs::SetContinualPlanningControl::Request::PAUSE;
        }

        loopSleep.sleep();
    }

    if(s_ContinualPlanning->isGoalFulfilled() || cpState == ContinualPlanning::FinishedAtGoal) {
        std::stringstream ss2;
        ss2 << "\n\nContinual planning ended.\n";
        if(s_ContinualPlanning->isGoalFulfilled())
            ss2 << "GOAL REACHED by agent!\n";
        if(cpState == ContinualPlanning::FinishedAtGoal)
            ss2 << "ContinualPlanningState: FinishedAtGoal!\n";
        ss2 << "\n";
        if(ros::ok())
            ROS_INFO("%s", ss2.str().c_str());
        else
            printf("%s", ss2.str().c_str());
    } else {
        if(ros::ok())
            ROS_ERROR("\n\nContinual planning ended.\nGOAL was NOT REACHED.\n\n");
        else
            printf("\n\nContinual planning ended.\nGOAL was NOT REACHED.\n\n\n");
    }

    return 0;
}

