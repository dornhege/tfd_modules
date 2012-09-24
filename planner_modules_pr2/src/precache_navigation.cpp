#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "planner_modules_pr2/module_param_cache.h"
#include "hardcoded_facts/geometryPoses.h"
#include "planner_modules_pr2/navstack_module.h"
#include <pluginlib/class_loader.h>
#include "continual_planning_executive/stateCreator.h"

static pluginlib::ClassLoader<continual_planning_executive::StateCreator>* s_StateCreatorLoader = NULL;
continual_planning_executive::StateCreator* s_StateCreatorRobotLocation = NULL;

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

bool fillRobotLocation(const SymbolicState & state, geometry_msgs::PoseStamped & ps)
{
    Predicate p;
    p.parameters.push_back("robot_location");
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

    // TODO sigint

    if(!loadStateCreator()) {
        ROS_FATAL("Failed to load state creator.");
        return 1;
    }

    // Read the params as set for tfd_modules and set them for us, as navstack_init will read those.
    nh.param("tfd_modules/trans_speed", g_TransSpeed, g_TransSpeed);
    nh.param("tfd_modules/rot_speed", g_RotSpeed, g_RotSpeed);
    ros::NodeHandle nhPriv("~");
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
    while(!s_StateCreatorRobotLocation->fillState(currentState)) {
        ROS_WARN("State estimation failed.");
        usleep(1000*1000);
    }
    ROS_INFO("State estimation done.");

    geometry_msgs::PoseStamped robotLocation;
    if(!fillRobotLocation(currentState, robotLocation)) {
        ROS_FATAL("Could not estimate robot location");
        return 1;
    }

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

    ROS_INFO("Precaching done.\n\n\n");

    return 0;
}
