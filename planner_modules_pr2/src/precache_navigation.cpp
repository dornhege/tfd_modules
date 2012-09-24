#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "planner_modules_pr2/module_param_cache.h"
#include "hardcoded_facts/geometryPoses.h"
#include "planner_modules_pr2/navstack_module.h"
#include <pluginlib/class_loader.h>
#include "continual_planning_executive/stateCreator.h"
#include "continual_planning_executive/goalCreator.h"

static pluginlib::ClassLoader<continual_planning_executive::StateCreator>* s_StateCreatorLoader = NULL;
continual_planning_executive::StateCreator* s_StateCreatorRobotLocation = NULL;
continual_planning_executive::StateCreator* s_StateCreatorRobotLocationInRoom = NULL;

static pluginlib::ClassLoader<continual_planning_executive::GoalCreator>* s_GoalCreatorLoader = NULL;
continual_planning_executive::GoalCreator* s_GoalCreator = NULL;

//ModuleParamCacheDouble g_PathCostCache;

bool loadStateCreator()
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

    // This should be name + params
    std::string state_creator_name = "tidyup_actions/state_creator_robot_pose";
    std::deque<std::string> state_creator_entry;
    state_creator_entry.push_back("robot_location");
    state_creator_entry.push_back("location");
    state_creator_entry.push_back("at-base");
    state_creator_entry.push_back("location");

    ROS_INFO("Loading state creator %s", state_creator_name.c_str());
    try {
        s_StateCreatorRobotLocation = s_StateCreatorLoader->createClassInstance(state_creator_name);
        s_StateCreatorRobotLocation->initialize(state_creator_entry);
    } catch(pluginlib::PluginlibException & ex) {
        ROS_ERROR("Failed to load StateCreator instance for: %s. Error: %s.",
                state_creator_name.c_str(), ex.what());
        return false;
    }

    state_creator_name = "tidyup_actions/state_creator_robot_location_in_room";
    state_creator_entry.clear();
    state_creator_entry.push_back("robot_location");
    state_creator_entry.push_back("location");

    ROS_INFO("Loading state creator %s", state_creator_name.c_str());
    try {
        s_StateCreatorRobotLocationInRoom = s_StateCreatorLoader->createClassInstance(state_creator_name);
        s_StateCreatorRobotLocationInRoom->initialize(state_creator_entry);
    } catch(pluginlib::PluginlibException & ex) {
        ROS_ERROR("Failed to load StateCreator instance for: %s. Error: %s.",
                state_creator_name.c_str(), ex.what());
        return false;
    }

    return true;
}

bool loadGoalCreator()
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

    // This should be name + params
    std::string goal_creator_name = "tidyup_actions/goal_creator_tidyup_objects";
    std::deque<std::string> goal_creator_entry;

    ROS_INFO("Loading goal creator %s", goal_creator_name.c_str());
    try {
        s_GoalCreator = s_GoalCreatorLoader->createClassInstance(goal_creator_name);
        s_GoalCreator->initialize(goal_creator_entry);
    } catch(pluginlib::PluginlibException & ex) {
        ROS_ERROR("Failed to load GoalCreator instance for: %s. Error: %s.",
                goal_creator_name.c_str(), ex.what());
        return false;
    }

    return true;
}


void precacheEntry(const std::string & start, const std::string & goal, geometry_msgs::PoseStamped startPose, geometry_msgs::PoseStamped goalPose)
{
    nav_msgs::GetPlan srv;
    nav_msgs::GetPlan::Request & request = srv.request;
    // create the path planning query for service
    request.start.header.frame_id = g_WorldFrame;
    request.goal.header.frame_id = g_WorldFrame;
    request.start.pose = startPose.pose;
    request.goal.pose = goalPose.pose;
    request.tolerance = g_GoalTolerance;

    // first lookup in the cache if we answered the query already
    double cost = modules::INFINITE_COST;
    std::string cacheKey = computePathCacheKey(start, goal, srv.request.start.pose, srv.request.goal.pose);
    if(g_PathCostCache.get(cacheKey, cost)) {
        ROS_INFO("Already got cost %.f for: %s", cost, cacheKey.c_str());
        return;
    }

    bool callSuccessful;
    cost = callPlanningService(srv, start, goal, callSuccessful);
    if(callSuccessful) {      // only cache real computed paths (including INFINITE_COST)
        //bool isRobotLocation =
        //    (parameterList[0].value == "robot_location" || parameterList[1].value == "robot_location");
        //g_PathCostCache.set(cacheKey, cost, !isRobotLocation);  // do no param cache robot_location calls
        ROS_INFO("Adding cache entry %s = %f", cacheKey.c_str(), cost);
        g_PathCostCache.set(cacheKey, cost, true);  // do param cache robot_location calls - they contain the location pose now (safe)
    }
}

bool fillPoseStamped(const std::string & name, const SymbolicState & state, geometry_msgs::PoseStamped & ps)
{
    Predicate p;
    p.parameters.push_back(name);
    p.name = "frame-id";
    if(!state.hasObjectFluent(p, &ps.header.frame_id))
        return false;
    p.name = "x";
    if(!state.hasNumericalFluent(p, &ps.pose.position.x))
        return false;
    p.name = "y";
    if(!state.hasNumericalFluent(p, &ps.pose.position.y))
        return false;
    p.name = "z";
    if(!state.hasNumericalFluent(p, &ps.pose.position.z))
        return false;
    p.name = "qx";
    if(!state.hasNumericalFluent(p, &ps.pose.orientation.x))
        return false;
    p.name = "qy";
    if(!state.hasNumericalFluent(p, &ps.pose.orientation.y))
        return false;
    p.name = "qz";
    if(!state.hasNumericalFluent(p, &ps.pose.orientation.z))
        return false;
    p.name = "qw";
    if(!state.hasNumericalFluent(p, &ps.pose.orientation.w))
        return false;

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "precache_navigation");
    ros::NodeHandle nh;
    tf::TransformListener tfl;

    if(argc != 2) {
        ROS_FATAL("Usage: %s <poses_file>", argv[0]);
        return 1;
    }

    ROS_INFO("Loading poses from %s", argv[1]);
    GeometryPoses posesLoader;
    if(!posesLoader.load(argv[1])) {
        ROS_FATAL("Failed to load poses.");
        return 1;
    }

    ros::NodeHandle nhPriv("~");
    // for the goal creator
    nhPriv.setParam("locations", argv[1]);

    // TODO sigint

    if(!loadStateCreator()) {
        ROS_FATAL("Failed to load state creator.");
        return 1;
    }
    if(!loadGoalCreator()) {
        ROS_FATAL("Failed to load goal creator.");
        return 1;
    }

    // Read the params as set for tfd_modules and set them for us, as navstack_init will read those.
    nh.param("tfd_modules/trans_speed", g_TransSpeed, g_TransSpeed);
    nh.param("tfd_modules/rot_speed", g_RotSpeed, g_RotSpeed);
    nhPriv.setParam("trans_speed", g_TransSpeed);
    nhPriv.setParam("rot_speed", g_RotSpeed);

    int navstack_argc = 4;
    char** navstack_argv = new char*[navstack_argc];
    navstack_argv[0] = "precache_navigation";
    navstack_argv[1] = "/map";
    navstack_argv[2] = "0.05";
    navstack_argv[3] = "1";

    navstack_init(navstack_argc, navstack_argv);

    delete[] navstack_argv;

    const std::map<std::string, geometry_msgs::PoseStamped> & poses = posesLoader.getPoses();

    SymbolicState currentState;
    SymbolicState goalState;

    s_GoalCreator->fillStateAndGoal(currentState, goalState);

    while(!s_StateCreatorRobotLocation->fillState(currentState)) {
        ROS_WARN("State estimation failed.");
        usleep(1000*1000);
    }
    while(!s_StateCreatorRobotLocationInRoom->fillState(currentState)) {
        ROS_WARN("State location in room estimation failed.");
        usleep(1000*1000);
    }
    ROS_INFO("State estimation done.");

    geometry_msgs::PoseStamped robotLocation;
    if(!fillPoseStamped("robot_location", currentState, robotLocation)) {
        ROS_FATAL("Could not estimate robot location");
        return 1;
    }

    Predicate robotLocationInRoom;
    robotLocationInRoom.name = "location-in-room";
    robotLocationInRoom.parameters.push_back("robot_location");
    std::string roomRobotLocation;
    if(!currentState.hasObjectFluent(robotLocationInRoom, &roomRobotLocation)) {
        ROS_ERROR("Could not determine room for robot_location");
    }

    const multimap<string, string> & typedObjects = currentState.getTypedObjects();
    for(multimap<string, string>::const_iterator it1 = typedObjects.begin(); it1 != typedObjects.end(); it1++) {
        //printf("At1 %s %s\n", it1->first.c_str(), it1->second.c_str());
        if(it1->first != "manipulation_location"
                && it1->first != "door_in_location"
                && it1->first != "door_out_location")
            continue;
        Predicate inRoom1;
        inRoom1.name = "location-in-room";
        inRoom1.parameters.push_back(it1->second);
        std::string room1;
        if(!currentState.hasObjectFluent(inRoom1, &room1))
            continue;
        geometry_msgs::PoseStamped pose1;
        if(!fillPoseStamped(it1->second, currentState, pose1)) {
            ROS_ERROR("Could not find pose for %s.", it1->second.c_str());
            continue;
        }

        for(multimap<string, string>::const_iterator it2 = typedObjects.begin(); it2 != typedObjects.end(); it2++){
            //printf("At2 %s %s\n", it2->first.c_str(), it2->second.c_str());
            if(it2->first != "manipulation_location"
                    && it2->first != "door_in_location"
                    && it2->first != "door_out_location")
                continue;
            if(it1->second == it2->second)
                continue;

            Predicate inRoom2;
            inRoom2.name = "location-in-room";
            inRoom2.parameters.push_back(it2->second);
            std::string room2;
            if(!currentState.hasObjectFluent(inRoom2, &room2))
                continue;

            if(room1 != room2)
                continue;
            if(it2->first == "door_out_location")
                continue;

            geometry_msgs::PoseStamped pose2;
            if(!fillPoseStamped(it2->second, currentState, pose2)) {
                ROS_ERROR("Could not find pose for %s.", it2->second.c_str());
                continue;
            }

            ROS_INFO("Precaching: %s - %s", it1->second.c_str(), it2->second.c_str());
            precacheEntry(it1->second, it2->second, pose1, pose2);
        }

        // Also add paths for robot_location
        if(room1 != roomRobotLocation)
            continue;
        if(it1->first == "door_out_location")
            continue;

        ROS_INFO("Precaching: %s - %s", "robot_location", it1->second.c_str());
        precacheEntry("robot_location", it1->second, robotLocation, pose1);
    }

    // through door?
    // robot location in room? for init robot_location caches
    // TODO test precache run quick!

    /*
    // location pairs
    for(std::map<std::string, geometry_msgs::PoseStamped>::const_iterator it1 = poses.begin(); it1 != poses.end(); it1++) {
        for(std::map<std::string, geometry_msgs::PoseStamped>::const_iterator it2 = poses.begin(); it2 != poses.end(); it2++) {
            if(it1->first == it2->first)
                continue;
            ROS_INFO("Precaching: %s - %s", it1->first.c_str(), it2->first.c_str());
            precacheEntry(it1->first, it2->first, it1->second, it2->second);
        }
    }

    // current location - location
    for(std::map<std::string, geometry_msgs::PoseStamped>::const_iterator it1 = poses.begin(); it1 != poses.end(); it1++) {
        ROS_INFO("Precaching: %s - %s", it1->first.c_str(), "robot_location");
        precacheEntry(it1->first, "robot_location", it1->second, robotLocation);
        ROS_INFO("Precaching: %s - %s", "robot_location", it1->first.c_str());
        precacheEntry("robot_location", it1->first, robotLocation, it1->second);
    }
    */
    // TODO only same room
    // TODO weird stuff happening
    // door in/out
    // door needs to be open in planning scene
    // please try only 1 goal pose
    // table1 pose looks off


    ROS_INFO("Precaching done.\n\n\n");

    return 0;
}
