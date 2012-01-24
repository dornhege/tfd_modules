#include <stdio.h>
#include <ros/ros.h>
#include <vector>
#include <deque>
#include <string>

#include "continual_planning_executive/symbolicState.h"
#include "continual_planning_executive/stateCreator.h"
#include "continual_planning_executive/goalCreator.h"
#include "continual_planning_executive/plannerInterface.h"
#include "planExecutor.h"
#include "continualPlanning.h"
#include <pluginlib/class_loader.h>

static ContinualPlanning s_ContinualPlanning;

static pluginlib::ClassLoader<continual_planning_executive::PlannerInterface>* s_PlannerLoader = NULL;
static pluginlib::ClassLoader<continual_planning_executive::StateCreator>* s_StateCreatorLoader = NULL;
static pluginlib::ClassLoader<continual_planning_executive::GoalCreator>* s_GoalCreatorLoader = NULL;
static pluginlib::ClassLoader<continual_planning_executive::ActionExecutorInterface>* s_ActionExecutorLoader = NULL;

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
        std::string state_creator_name = xmlRpc[i];
        ROS_INFO("Loading state creator %s", state_creator_name.c_str());
        try {
            continual_planning_executive::StateCreator* sc = s_StateCreatorLoader->createClassInstance(state_creator_name);
            s_ContinualPlanning._stateCreators.push_back(sc);
        } catch(pluginlib::PluginlibException & ex) {
            ROS_ERROR("Failed to load StateCreator instance for: %s. Error: %s.",
                    state_creator_name.c_str(), ex.what());
            return false;
        }
    }
    return !s_ContinualPlanning._stateCreators.empty();
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
        std::string goal_creator_name = xmlRpc[i];
        ROS_INFO("Loading goal creator %s", goal_creator_name.c_str());
        try {
            continual_planning_executive::GoalCreator* gc = s_GoalCreatorLoader->createClassInstance(goal_creator_name);
            if(!gc->fillStateAndGoal(s_ContinualPlanning._currentState, s_ContinualPlanning._goal)) {
                ROS_ERROR("Filling state and goal failed for goal_creator %s.", goal_creator_name.c_str());
                return false;
            }
        } catch(pluginlib::PluginlibException & ex) {
            ROS_ERROR("Failed to load GoalCreator instance for: %s. Error: %s.",
                    goal_creator_name.c_str(), ex.what());
            return false;
        }
    }

    ROS_INFO_STREAM("Goal initialized to:\n" << s_ContinualPlanning._goal);
    return true;
}

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
            s_ContinualPlanning._planExecutor.addActionExecutor(ae);
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
        s_ContinualPlanning._planner = s_PlannerLoader->createClassInstance(planner_name);
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

    // init planner
    s_ContinualPlanning._planner->initialize(domainFile, plannerOptions);

    return s_ContinualPlanning._planner != NULL;
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

int main(int argc, char** argv)
{
    ROS_INFO("Continual Planning Executive started.");

    ros::init(argc, argv, "continual_planning_executive");

    ros::NodeHandle nh;

    if(!init()) {
        ROS_FATAL("Init failed.");
        return 1;
    }

    ros::Rate loopSleep(5);
    ContinualPlanning::ContinualPlanningState cpState = ContinualPlanning::Running;
    while(ros::ok()) {
        ros::spinOnce();

        cpState = s_ContinualPlanning.loop();
        if(cpState != ContinualPlanning::Running) {
            break;
        }

        loopSleep.sleep();
    }

    if(s_ContinualPlanning.isGoalFulfilled() || cpState == ContinualPlanning::FinishedAtGoal) {
        if(ros::ok())
            ROS_INFO("\n\nContinual planning ended.\nGOAL REACHED by agent!\n\n");
        else
            printf("\n\nContinual planning ended.\nGOAL REACHED by agent!\n\n\n");
    } else {
        if(ros::ok())
            ROS_ERROR("\n\nContinual planning ended.\nGOAL was NOT REACHED.\n\n");
        else
            printf("\n\nContinual planning ended.\nGOAL was NOT REACHED.\n\n\n");
    }

    return 0;
}

