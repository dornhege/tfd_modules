#include "navstack_module_full_body.h"
#include "navstack_module.h"
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

VERIFY_CONDITIONCHECKER_DEF(fullbody_pathCost);

static ros::ServiceClient s_SwitchJointTopicClient;
static ros::Subscriber s_JointStateSubscriber;
static ros::Publisher s_PlanningJointStatePublisher;
static sensor_msgs::JointState s_CurrentState;
static sensor_msgs::JointState s_RightArmAtSide;
static sensor_msgs::JointState s_LeftArmAtSide;

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
    navstack_init(argc, argv);

    // init service query for joint topic switcher
    std::string service_name = "/mux_joint_states/select";
    while(!ros::service::waitForService(service_name, ros::Duration(3.0))) {
        ROS_ERROR("Service %s not available - waiting.", service_name.c_str());
    }
    s_SwitchJointTopicClient = g_NodeHandle->serviceClient<topic_tools::MuxSelect>(service_name, true);
    if(!s_SwitchJointTopicClient) {
        ROS_FATAL("Could not initialize get plan service from %s (client name: %s)", service_name.c_str(), s_SwitchJointTopicClient.getService().c_str());
    }

    // init joint state publisher
    s_JointStateSubscriber = g_NodeHandle->subscribe("/joint_states_throttle", 3, jointStateCallback);
    s_PlanningJointStatePublisher = g_NodeHandle->advertise<sensor_msgs::JointState>("/joint_states_tfd", 5, false);

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
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
    if(g_Debug) {        // prevent spamming ROS_DEBUG calls unless we really want debug
        // debugging raw planner calls
        static unsigned int calls = 0;
        calls++;
        if(calls % 10000 == 0) {
            ROS_DEBUG("Got %d module calls.\n", calls);
        }
    }

    // first lookup in the cache if we answered the query already
    map<pair<string, string>, double>::iterator it = g_PathCostCache.find(make_pair(parameterList[0].value, parameterList[1].value));
    if(it != g_PathCostCache.end()) {
        return it->second;
    }

    nav_msgs::GetPlan srv;
    if(!fillPathRequest(parameterList, numericalFluentCallback, srv.request)) {
        return INFINITE_COST;
    }

    double cost = INFINITE_COST;

    if(!g_GetPlan) {
        ROS_ERROR("Persistent service connection to %s failed.", g_GetPlan.getService().c_str());
        // FIXME reconnect - this shouldn't happen.
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
retryGetPlan:
    static unsigned int failCounter = 0;

    // perform the actual path planner call
    if(g_GetPlan.call(srv)) {
        failCounter = 0;

        if(g_Debug) {
            ros::Time callEndTime = ros::Time::now();
            ros::Duration dt = callEndTime - callStartTime;
            totalCallsTime += dt;
            ROS_DEBUG("ServiceCall took: %f, avg: %f (num %f).", dt.toSec(), totalCallsTime.toSec()/plannerCalls, plannerCalls);
        }

        if(!srv.response.plan.poses.empty()) {
            // get plan cost
            double pathLength = 0;
            geometry_msgs::PoseStamped lastPose = srv.response.plan.poses[0];
            forEach(const geometry_msgs::PoseStamped & p, srv.response.plan.poses) {
                double d = hypot(lastPose.pose.position.x - p.pose.position.x,
                    lastPose.pose.position.y - p.pose.position.y);
            pathLength += d;
            lastPose = p;
        }
            cost = pathLength;
        } else {
            ROS_WARN("Got empty plan: %s -> %s", parameterList[0].value.c_str(), parameterList[1].value.c_str());
        }

        //ROS_INFO("Got plan: %s -> %s cost: %f.", parameterList[0].value.c_str(), parameterList[1].value.c_str(), cost);
        // also empty plan = OK or fail or none?
    } else {
        ROS_ERROR("Failed to call service %s - is the robot moving?", g_GetPlan.getService().c_str());
        failCounter++;
        if(failCounter < 300) {
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
    g_PathCostCache[make_pair(parameterList[0].value, parameterList[1].value)] = cost;

    // switch joint topic to execution mode
    switchSrv.request.topic = switchSrv.response.prev_topic;
    s_SwitchJointTopicClient.call(switchSrv);
    return cost;
}

