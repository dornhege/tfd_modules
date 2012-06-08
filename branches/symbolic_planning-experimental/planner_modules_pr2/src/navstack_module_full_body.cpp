#include "navstack_module_full_body.h"
#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <topic_tools/MuxSelect.h>
#include <map>
using std::map;
#include <utility>
using std::pair; using std::make_pair;
#include <boost/foreach.hpp>
#ifdef __CDT_PARSER__
    #define forEach(a, b) for(a : b)
#else
    #define forEach BOOST_FOREACH
#endif
#include <sys/times.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

/**
 * Simple module implementation for ROS navigation stack.
 *
 * Directly queries the move_base_node/make_plan service for each
 * cost request by the planner.
 */

static const bool s_Debug = false;

static ros::NodeHandle* s_NodeHandle = NULL;
static ros::ServiceClient s_GetPlan;
static ros::ServiceClient s_SwitchJointTopicClient;
static ros::Subscriber s_JointStateSubscriber;
static ros::Publisher s_PlanningJointStatePublisher;
static sensor_msgs::JointState s_CurrentState;
static sensor_msgs::JointState s_RightArmAtSide;
static sensor_msgs::JointState s_LeftArmAtSide;

/// Plan requests are issued using this frame - so the poses from the planner are given in this frame (e.g. map)
static std::string s_WorldFrame;

static double s_GoalTolerance = 0.5;

// Using a cache of queried path costs to prevent calling the path planning service multiple times
// Better: Can we assume symmetric path costs?
static map< pair<string,string>, double> s_PathCostCache;

void jointStateCallback(const sensor_msgs::JointState& msg)
{
    ROS_WARN_STREAM("new joint state: " << msg);
    s_CurrentState = msg;
    ROS_WARN_STREAM("new joint state: " << s_CurrentState);
}

void replaceJointPosition(sensor_msgs::JointState& oldState, sensor_msgs::JointState& newJoints)
{
    for (unsigned int i = 0; i < newJoints.name.size(); i++)
    {
        string name = newJoints.name[i];
        int index = -1;
        for (unsigned int j = 0; j < oldState.name.size(); j++)
        {
            if (name.compare(oldState.name[j]) == 0)
            {
                index = j;
            }
        }
        if (index != -1)
        {
            oldState.position[index] = newJoints.position[i];
        }
    }
}

void fullbody_navstack_init(int argc, char** argv)
{
    ROS_ASSERT(argc == 4);

    // get world frame
    ros::NodeHandle nhPriv("~");
    std::string tfPrefix = tf::getPrefixParam(nhPriv);
    s_WorldFrame = tf::resolve(tfPrefix, argv[1]);
    ROS_INFO("World frame is: %s", s_WorldFrame.c_str());

    // get goal tolerance
    char* checkPtr;
    s_GoalTolerance = strtod(argv[2], &checkPtr);
    if (checkPtr == argv[2])
    { // conversion error!
        ROS_ERROR("%s: Could not convert argument for goal tolerance: %s", __func__, argv[2]);
        s_GoalTolerance = 0.5;
    }

    if (strcmp(argv[3], "0") == 0)
    {
        ROS_INFO("Using absolute goal tolerance.");
    }
    else
    { // relative goal tolerance, argv[3] contains the base_local_planner namespace
        ROS_INFO("Using relative goal tolerance.");
        // get base_local_planner's xy_goal_tolerance
        std::string base_local_planner_ns = argv[3];
        double move_base_tol;
        ros::NodeHandle nh;
        if (!nh.getParam(base_local_planner_ns + "/xy_goal_tolerance", move_base_tol))
        {
            ROS_ERROR_STREAM("requested relative goal tolerance, but " << (base_local_planner_ns + "/xy_goal_tolerance") << " was not set" << " - falling back to absolute mode");
        }
        else
        { // 2. add move_base's tolerance to our relative tolerance
            s_GoalTolerance += move_base_tol;
        }
    }

    ROS_INFO("Goal Tolerance is: %f.", s_GoalTolerance);

    // init service query for make plan
    string service_name = "move_base/make_plan";
    s_NodeHandle = new ros::NodeHandle();
    while (!ros::service::waitForService(service_name, ros::Duration(3.0)))
    {
        ROS_ERROR("Service %s not available - waiting.", service_name.c_str());
    }
    s_GetPlan = s_NodeHandle->serviceClient<nav_msgs::GetPlan>(service_name, true);
    if (!s_GetPlan)
    {
        ROS_FATAL("Could not initialize get plan service from %s (client name: %s)", service_name.c_str(), s_GetPlan.getService().c_str());
    }

    // init service query for joint topic switcher
    service_name = "/mux_joint_states/select";
    while (!ros::service::waitForService(service_name, ros::Duration(3.0)))
    {
        ROS_ERROR("Service %s not available - waiting.", service_name.c_str());
    }
    s_SwitchJointTopicClient = s_NodeHandle->serviceClient<topic_tools::MuxSelect>(service_name, true);
    if (!s_GetPlan)
    {
        ROS_FATAL("Could not initialize get plan service from %s (client name: %s)", service_name.c_str(), s_GetPlan.getService().c_str());
    }

    // init joint state publisher
    s_JointStateSubscriber = s_NodeHandle->subscribe("/joint_states_throttle", 3, jointStateCallback);
    s_PlanningJointStatePublisher = s_NodeHandle->advertise<sensor_msgs::JointState>("/joint_states_tfd", 5, false);

    // init joint state messages for possible arm states
    // right arm at side
    s_RightArmAtSide.name.push_back("r_shoulder_pan_joint");
    s_RightArmAtSide.name.push_back("r_shoulder_lift_joint");
    s_RightArmAtSide.name.push_back("r_upper_arm_roll_joint");
    s_RightArmAtSide.name.push_back("r_elbow_flex_joint");
    s_RightArmAtSide.name.push_back("r_forearm_roll_joint");
    s_RightArmAtSide.name.push_back("r_wrist_flex_joint");
    s_RightArmAtSide.name.push_back("r_wrist_roll_joint");
    // [-2.110, 1.230, -2.06, -1.69, 3.439, -1.52, 1.57]
    s_RightArmAtSide.position.push_back(-2.110);
    s_RightArmAtSide.position.push_back(1.230);
    s_RightArmAtSide.position.push_back(-2.06);
    s_RightArmAtSide.position.push_back(-1.69);
    s_RightArmAtSide.position.push_back(3.439);
    s_RightArmAtSide.position.push_back(-1.52);
    s_RightArmAtSide.position.push_back(1.57);
    // left arm at side
    s_LeftArmAtSide.name.push_back("l_shoulder_pan_joint");
    s_LeftArmAtSide.name.push_back("l_shoulder_lift_joint");
    s_LeftArmAtSide.name.push_back("l_upper_arm_roll_joint");
    s_LeftArmAtSide.name.push_back("l_elbow_flex_joint");
    s_LeftArmAtSide.name.push_back("l_forearm_roll_joint");
    s_LeftArmAtSide.name.push_back("l_wrist_flex_joint");
    s_LeftArmAtSide.name.push_back("l_wrist_roll_joint");
    // [2.110, 1.230, 2.06, -1.69, -3.439, -1.52, 1.57]
    s_LeftArmAtSide.position.push_back(2.110);
    s_LeftArmAtSide.position.push_back(1.230);
    s_LeftArmAtSide.position.push_back(2.06);
    s_LeftArmAtSide.position.push_back(-1.69);
    s_LeftArmAtSide.position.push_back(-3.439);
    s_LeftArmAtSide.position.push_back(-1.52);
    s_LeftArmAtSide.position.push_back(1.57);

    ROS_INFO("Initialized Navstack Module.\n");
}

double fullbody_pathCost(const ParameterList & parameterList,
        predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
    if (s_Debug)
    { // prevent spamming ROS_DEBUG calls unless we really want debug
        // debugging raw planner calls
        static unsigned int calls = 0;
        calls++;
        if (calls % 10000 == 0)
        {
            ROS_DEBUG("Got %d module calls.\n", calls);
        }
    }

    // first lookup in the cache if we answered the query already
    map<pair<string, string>, double>::iterator it = s_PathCostCache.find(
            make_pair(parameterList[0].value, parameterList[1].value));
    if (it != s_PathCostCache.end())
    {
        return it->second;
    }

    // get robot and target location from planner interface
    ROS_ASSERT(parameterList.size() == 2);

    ParameterList startParams;
    startParams.push_back(parameterList[0]);
    ParameterList goalParams;
    goalParams.push_back(parameterList[1]);
    NumericalFluentList nfRequest;
    nfRequest.reserve(14);
    nfRequest.push_back(NumericalFluent("x", startParams));
    nfRequest.push_back(NumericalFluent("y", startParams));
    nfRequest.push_back(NumericalFluent("z", startParams));
    nfRequest.push_back(NumericalFluent("qx", startParams));
    nfRequest.push_back(NumericalFluent("qy", startParams));
    nfRequest.push_back(NumericalFluent("qz", startParams));
    nfRequest.push_back(NumericalFluent("qw", startParams));
    nfRequest.push_back(NumericalFluent("x", goalParams));
    nfRequest.push_back(NumericalFluent("y", goalParams));
    nfRequest.push_back(NumericalFluent("z", goalParams));
    nfRequest.push_back(NumericalFluent("qx", goalParams));
    nfRequest.push_back(NumericalFluent("qy", goalParams));
    nfRequest.push_back(NumericalFluent("qz", goalParams));
    nfRequest.push_back(NumericalFluent("qw", goalParams));

    NumericalFluentList* nfRequestP = &nfRequest;
    if (!numericalFluentCallback(nfRequestP))
    {
        ROS_ERROR("numericalFluentCallback failed.");
        return INFINITE_COST;
    }

    // create the path planning query for service
    nav_msgs::GetPlan srv;
    srv.request.start.header.frame_id = s_WorldFrame;
    srv.request.goal.header.frame_id = s_WorldFrame;
    srv.request.start.pose.position.x = nfRequest[0].value;
    srv.request.start.pose.position.y = nfRequest[1].value;
    srv.request.start.pose.position.z = nfRequest[2].value;
    srv.request.start.pose.orientation.x = nfRequest[3].value;
    srv.request.start.pose.orientation.y = nfRequest[4].value;
    srv.request.start.pose.orientation.z = nfRequest[5].value;
    srv.request.start.pose.orientation.w = nfRequest[6].value;
    srv.request.goal.pose.position.x = nfRequest[7].value;
    srv.request.goal.pose.position.y = nfRequest[8].value;
    srv.request.goal.pose.position.z = nfRequest[9].value;
    srv.request.goal.pose.orientation.x = nfRequest[10].value;
    srv.request.goal.pose.orientation.y = nfRequest[11].value;
    srv.request.goal.pose.orientation.z = nfRequest[12].value;
    srv.request.goal.pose.orientation.w = nfRequest[13].value;
    srv.request.tolerance = s_GoalTolerance;

    double cost = INFINITE_COST;

    if (!s_GetPlan)
    {
        ROS_ERROR("Persistent service connection to %s failed.", s_GetPlan.getService().c_str());
        return INFINITE_COST;
    }

    if (!s_SwitchJointTopicClient)
    {
        ROS_ERROR("Persistent service connection to %s failed.", s_SwitchJointTopicClient.getService().c_str());
        return INFINITE_COST;
    }

    topic_tools::MuxSelect switchSrv;
    switchSrv.request.topic = "/joint_states_tfd";
    if (s_SwitchJointTopicClient.call(switchSrv))
    {
//		s_RightArmAtSide.header.stamp = ros::Time::now();
//		s_LeftArmAtSide.header.stamp = s_RightArmAtSide.header.stamp;
//		s_PlanningJointStatePublisher.publish(s_RightArmAtSide);
//		s_PlanningJointStatePublisher.publish(s_LeftArmAtSide);
        replaceJointPosition(s_CurrentState, s_RightArmAtSide);
        replaceJointPosition(s_CurrentState, s_LeftArmAtSide);
        s_PlanningJointStatePublisher.publish(s_CurrentState);
		ROS_INFO("Publishing planning arm states...");
    }

    // statistics about using the ros path planner service
    static double plannerCalls = 0;
    static ros::Duration totalCallsTime = ros::Duration(0.0);
    plannerCalls += 1.0;

    ros::Time callStartTime = ros::Time::now();
    // This construct is here, because when the robot is moving move_base will not produce other paths
    // we retry for a certain amount of time to not fail directly.
    // FIXME: Cleanup the goto code
    retryGetPlan: static unsigned int failCounter = 0;

    // perform the actual path planner call
    if (s_GetPlan.call(srv))
    {
        failCounter = 0;

        if (s_Debug)
        {
            ros::Time callEndTime = ros::Time::now();
            ros::Duration dt = callEndTime - callStartTime;
            totalCallsTime += dt;
            ROS_DEBUG("ServiceCall took: %f, avg: %f (num %f).", dt.toSec(), totalCallsTime.toSec()/plannerCalls, plannerCalls);
        }

        if (!srv.response.plan.poses.empty())
        {
            // get plan cost
            double pathLength = 0;
            geometry_msgs::PoseStamped lastPose = srv.response.plan.poses[0];
            forEach(const geometry_msgs::PoseStamped & p, srv.response.plan.poses){
            double d = hypot(lastPose.pose.position.x - p.pose.position.x,
                    lastPose.pose.position.y - p.pose.position.y);
            pathLength += d;
            lastPose = p;
        }
            cost = pathLength;
        }
        else
        {
            ROS_WARN("Got empty plan: %s -> %s", parameterList[0].value.c_str(), parameterList[1].value.c_str());
        }

        //ROS_INFO("Got plan: %s -> %s cost: %f.", parameterList[0].value.c_str(), parameterList[1].value.c_str(), cost);
        // also empty plan = OK or fail or none?
    }
    else
    {
        ROS_ERROR("Failed to call service %s - is the robot moving?", s_GetPlan.getService().c_str());
        failCounter++;
        if (failCounter < 300)
        {
            usleep(1000 * 1000);
            goto retryGetPlan;
        }

        // switch joint topic to execution mode
        switchSrv.request.topic = switchSrv.response.prev_topic;
        s_SwitchJointTopicClient.call(switchSrv);
        // FIXME: what if target is unreachable, do we get false or an empty plan? i.e. is this an error
        return INFINITE_COST;
    }

    // return pathcost and cache
    s_PathCostCache[make_pair(parameterList[0].value, parameterList[1].value)] = cost;

    // switch joint topic to execution mode
    switchSrv.request.topic = switchSrv.response.prev_topic;
    s_SwitchJointTopicClient.call(switchSrv);
    return cost;
}

