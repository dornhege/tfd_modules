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
#include "continual_planning_executive/SetContinualPlanningControl.h"
#include "continual_planning_executive/ExecuteActionDirectly.h"
#include "planExecutor.h"
#include "continualPlanning.h"
#include <pluginlib/class_loader.h>

#include <QString>
#include <ros/package.h>

static ContinualPlanning* s_ContinualPlanning = NULL;

static pluginlib::ClassLoader<continual_planning_executive::PlannerInterface>* s_PlannerLoader = NULL;
static pluginlib::ClassLoader<continual_planning_executive::StateCreator>* s_StateCreatorLoader = NULL;
static pluginlib::ClassLoader<continual_planning_executive::GoalCreator>* s_GoalCreatorLoader = NULL;
static pluginlib::ClassLoader<continual_planning_executive::ActionExecutorInterface>* s_ActionExecutorLoader = NULL;

static int s_ContinualPlanningMode = continual_planning_executive::SetContinualPlanningControl::Request::RUN;

std::deque<std::string> splitString(const std::string & s, const char* delim)
{
    std::deque<std::string> elems;
    std::string::size_type lastPos = 0;
    std::string::size_type pos     = 0;

    do {
        pos = s.find_first_of(delim, lastPos);
        elems.push_back(s.substr(lastPos, pos - lastPos));
        lastPos = pos + 1;
    } while(std::string::npos != pos);

    return elems;
}

bool loadStateCreators(ros::NodeHandle & nh)
{
    try {
        // create here with new as it can't go out of scope
        s_StateCreatorLoader 
            = new pluginlib::ClassLoader<continual_planning_executive::StateCreator>
            ("continual_planning_executive", "continual_planning_executive::StateCreator");
    } catch(pluginlib::PluginlibException & ex) {
        // possible reason for failure: no known plugins
        ROS_ERROR("Could not instantiate class loader for continual_planning_executive::StateCreator - are there plugins registered? Error: %s", ex.what());
        return false;
    }

    XmlRpc::XmlRpcValue xmlRpc;
    if(!nh.getParam("state_creators", xmlRpc)) {
        ROS_ERROR("No state_creators defined.");
        return false;
    }
    if(xmlRpc.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("state_creators param should be a list.");
        return false;
    }
    if(xmlRpc.size() == 0) {
        ROS_ERROR("state_creators list is empty.");
        return false;
    }
    for(int i = 0; i < xmlRpc.size(); i++) {
        if(xmlRpc[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("state_creators entry %d is not of type string.", i);
            return false;
        }
        // This should be name + params
        std::deque<std::string> state_creator_entry = splitString(xmlRpc[i], " ");
        ROS_ASSERT(state_creator_entry.size() >= 1);

        std::string state_creator_name = state_creator_entry.at(0);
        state_creator_entry.pop_front();  // no only params left

        ROS_INFO("Loading state creator %s", state_creator_name.c_str());
        try {
            continual_planning_executive::StateCreator* sc = s_StateCreatorLoader->createClassInstance(state_creator_name);
            sc->initialize(state_creator_entry);
            s_ContinualPlanning->_stateCreators.push_back(sc);
        } catch(pluginlib::PluginlibException & ex) {
            ROS_ERROR("Failed to load StateCreator instance for: %s. Error: %s.",
                    state_creator_name.c_str(), ex.what());
            return false;
        }
    }

    return !s_ContinualPlanning->_stateCreators.empty();
}

bool loadGoalCreators(ros::NodeHandle & nh)
{
    try {
        // create here with new as it can't go out of scope
        s_GoalCreatorLoader
            = new pluginlib::ClassLoader<continual_planning_executive::GoalCreator>
            ("continual_planning_executive", "continual_planning_executive::GoalCreator");
    } catch(pluginlib::PluginlibException & ex) {
        // possible reason for failure: no known plugins
        ROS_ERROR("Could not instantiate class loader for continual_planning_executive::GoalCreator - are there plugins registered? Error: %s", ex.what());
        return false;
    }

    XmlRpc::XmlRpcValue xmlRpc;
    if(!nh.getParam("goal_creators", xmlRpc)) {
        ROS_ERROR("No goal_creators defined.");
        return false;
    } 
    if(xmlRpc.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("goal_creators param should be a list.");
        return false;
    }
    if(xmlRpc.size() == 0) {
        ROS_ERROR("goal_creators list is empty.");
        return false;
    }
    for(int i = 0; i < xmlRpc.size(); i++) {
        if(xmlRpc[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("goal_creators entry %d is not of type string.", i);
            return false;
        }
        // This should be name + params
        std::deque<std::string> goal_creator_entry = splitString(xmlRpc[i], " ");
        ROS_ASSERT(goal_creator_entry.size() >= 1);

        std::string goal_creator_name = goal_creator_entry.at(0);
        goal_creator_entry.pop_front();  // no only params left

        ROS_INFO("Loading goal creator %s", goal_creator_name.c_str());
        try {
            continual_planning_executive::GoalCreator* gc = s_GoalCreatorLoader->createClassInstance(goal_creator_name);
            gc->initialize(goal_creator_entry);
            if(!gc->fillStateAndGoal(s_ContinualPlanning->_currentState, s_ContinualPlanning->_goal)) {
                ROS_ERROR("Filling state and goal failed for goal_creator %s.", goal_creator_name.c_str());
                return false;
            }
        } catch(pluginlib::PluginlibException & ex) {
            ROS_ERROR("Failed to load GoalCreator instance for: %s. Error: %s.",
                    goal_creator_name.c_str(), ex.what());
            return false;
        }
    }

    ROS_INFO_STREAM("Goal initialized to:\n" << s_ContinualPlanning->_goal);
    return true;
}

bool loadActionExecutors(ros::NodeHandle & nh)
{
    try {
        // create here with new as it can't go out of scope
        s_ActionExecutorLoader
            = new pluginlib::ClassLoader<continual_planning_executive::ActionExecutorInterface>
            ("continual_planning_executive", "continual_planning_executive::ActionExecutorInterface");
    } catch(pluginlib::PluginlibException & ex) {
        // possible reason for failure: no known plugins
        ROS_ERROR("Could not instantiate class loader for continual_planning_executive::ActionExecutorInterface - are there plugins registered? Error: %s", ex.what());
        return false;
    }

    XmlRpc::XmlRpcValue xmlRpc;
    if(!nh.getParam("action_executors", xmlRpc)) {
        ROS_ERROR("No action_executors defined.");
        return false;
    } 
    if(xmlRpc.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("action_executors param should be a list.");
        return false;
    }
    if(xmlRpc.size() == 0) {
        ROS_ERROR("action_executors list is empty.");
        return false;
    }
    for(int i = 0; i < xmlRpc.size(); i++) {
        if(xmlRpc[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("action_executors entry %d is not of type string.", i);
            return false;
        }
        // This should be name + params
        std::deque<std::string> action_executor_entry = splitString(xmlRpc[i], " ");
        ROS_ASSERT(action_executor_entry.size() >= 1);

        std::string action_executor_name = action_executor_entry.at(0);
        action_executor_entry.pop_front();  // no only params left

        ROS_INFO("Loading action_executor %s", action_executor_name.c_str());
        try {
            continual_planning_executive::ActionExecutorInterface* ae
                = s_ActionExecutorLoader->createClassInstance(action_executor_name);
            ae->initialize(action_executor_entry);
            s_ContinualPlanning->_planExecutor.addActionExecutor(ae);
        } catch(pluginlib::PluginlibException & ex) {
            ROS_ERROR("Failed to load ActionExecutor instance for: %s. Error: %s.",
                    action_executor_name.c_str(), ex.what());
            return false;
        }
    }
    return true;
}

bool loadPlanner(ros::NodeHandle & nh)
{
    // load planner
    try {
        // create here with new as it can't go out of scope
        s_PlannerLoader
            = new pluginlib::ClassLoader<continual_planning_executive::PlannerInterface>
            ("continual_planning_executive", "continual_planning_executive::PlannerInterface");
    } catch(pluginlib::PluginlibException & ex) {
        // possible reason for failure: no known plugins
        ROS_ERROR("Could not instantiate class loader for continual_planning_executive::PlannerInterface - are there plugins registered? Error: %s", ex.what());
        return false;
    }

    std::string planner_name;
    if(!nh.getParam("planner", planner_name)) {
        ROS_ERROR("No planner defined!");
        return false;
    }
    ROS_INFO("Loading planner %s", planner_name.c_str());
    try {
        s_ContinualPlanning->_planner = s_PlannerLoader->createClassInstance(planner_name);
    } catch(pluginlib::PluginlibException & ex) {
        ROS_ERROR("Failed to load Planner instance for: %s. Error: %s.",
                planner_name.c_str(), ex.what());
        return false;
    }

    // load planner options
    std::vector<std::string> plannerOptions;
    XmlRpc::XmlRpcValue xmlRpc;
    if(!nh.getParam("planner_options", xmlRpc)) {
        ROS_INFO("No planner_options defined.");
    } else {
        if(xmlRpc.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("planner_options param should be a list.");
            return false;
        }
        for(int i = 0; i < xmlRpc.size(); i++) {
            if(xmlRpc[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("planner_options entry %d is not of type string.", i);
                return false;
            }
            plannerOptions.push_back(xmlRpc[i]);
            ROS_DEBUG("Adding planner option \"%s\"", plannerOptions.back().c_str());
        }
    } 

    // get domain
    std::string domainFile;
    if(!nh.getParam("domain_file", domainFile)) {
        ROS_ERROR("Could not get ~domain_file parameter.");
        return false;
    }
    QString qDomainFile = QString::fromStdString(domainFile);
    QString package_prefix("package://");
    if (qDomainFile.startsWith(package_prefix))
    {
        int end_of_package = qDomainFile.indexOf('/', package_prefix.length());
        QString package(qDomainFile.mid(package_prefix.length(), end_of_package - package_prefix.length()));
        domainFile = ros::package::getPath(package.toStdString());
        domainFile.append(qDomainFile.mid(end_of_package).toStdString());
    }
    ROS_INFO_STREAM("domain file: " << domainFile);

    // init planner
    s_ContinualPlanning->_planner->initialize(domainFile, plannerOptions);

    return s_ContinualPlanning->_planner != NULL;
}

bool init()
{
    ros::NodeHandle nhPriv("~");

    // planner
    if(!loadPlanner(nhPriv))
        return false;
    ROS_INFO("Loaded planner.");

    // state creators
    if(!loadStateCreators(nhPriv))
        return false;
    ROS_INFO("Loaded state creators.");

    // goal
    if(!loadGoalCreators(nhPriv))
        return false;
    ROS_INFO("Loaded goal creators.");

    // actions
    if(!loadActionExecutors(nhPriv))
        return false;
    ROS_INFO("Loaded action executors.");

    return true;
}

bool setControlHandler(continual_planning_executive::SetContinualPlanningControl::Request & req,
        continual_planning_executive::SetContinualPlanningControl::Response & resp)
{
    switch(req.command) {
        case continual_planning_executive::SetContinualPlanningControl::Request::RUN:
        case continual_planning_executive::SetContinualPlanningControl::Request::PAUSE:
        case continual_planning_executive::SetContinualPlanningControl::Request::STEP:
            if(s_ContinualPlanningMode != req.command) {
                ROS_INFO("Setting ContinualPlanningMode to %d", req.command);
            }
            s_ContinualPlanningMode = req.command;
            resp.command = s_ContinualPlanningMode;
            break;
        case continual_planning_executive::SetContinualPlanningControl::Request::FORCE_REPLANNING:
            s_ContinualPlanning->forceReplanning();
            resp.command = req.command;
            break;
        case continual_planning_executive::SetContinualPlanningControl::Request::REESTIMATE_STATE:
            if(!s_ContinualPlanning->estimateCurrentState()) {
                ROS_WARN("SetContinualPlanningControl: State estimation failed.");
                return false;
            }
            resp.command = req.command;
            break;
        default:
            ROS_ERROR("Invalid command in continual_planning_executive::SetContinualPlanningControl: %d",
                    req.command);
            return false;
    }
    return true;
}

bool executeActionDirectlyHandler(continual_planning_executive::ExecuteActionDirectly::Request & req,
        continual_planning_executive::ExecuteActionDirectly::Response & resp)
{
    if(s_ContinualPlanningMode == continual_planning_executive::SetContinualPlanningControl::Request::RUN) {
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

    if(!init()) {
        ROS_FATAL("Init failed.");
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
        s_ContinualPlanningMode = continual_planning_executive::SetContinualPlanningControl::Request::PAUSE;

    ros::Rate loopSleep(5);
    ContinualPlanning::ContinualPlanningState cpState = ContinualPlanning::Running;
    while(ros::ok()) {
        ros::spinOnce();

        if(s_ContinualPlanningMode == continual_planning_executive::SetContinualPlanningControl::Request::RUN
            || s_ContinualPlanningMode == continual_planning_executive::SetContinualPlanningControl::Request::STEP) {
            cpState = s_ContinualPlanning->loop();
            if(cpState != ContinualPlanning::Running) {
                break;
            }
            // STEP means RUN once.
            if(s_ContinualPlanningMode == continual_planning_executive::SetContinualPlanningControl::Request::STEP)
                s_ContinualPlanningMode = continual_planning_executive::SetContinualPlanningControl::Request::PAUSE;
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

