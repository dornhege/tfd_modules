#include "planner_modules_pr2/navstack_module_full_body.h"
#include "planner_modules_pr2/navstack_module.h"
#include "tidyup_utils/arm_state.h"
#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
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
bool receivedJointState;

void jointStateCallback(const sensor_msgs::JointState& msg)
{
    receivedJointState = true;
    s_CurrentState = msg;
}

void publishPlanningArmState()
{
    if (!s_SwitchJointTopicClient)
    {
        ROS_ERROR("Persistent service connection to %s failed.", s_SwitchJointTopicClient.getService().c_str());
        return;
    }
    topic_tools::MuxSelect switchSrv;
    switchSrv.request.topic = "/joint_states_tfd";
    if (s_SwitchJointTopicClient.call(switchSrv))
    {
        ROS_INFO("%s: switched to topic \"%s\".", __FUNCTION__, switchSrv.request.topic.c_str());
        ArmState::get("/arm_configurations/side_tuck/position/", "right_arm").replaceJointPositions(s_CurrentState);
        ArmState::get("/arm_configurations/side_tuck/position/", "left_arm").replaceJointPositions(s_CurrentState);
        s_PlanningJointStatePublisher.publish(s_CurrentState);
        ROS_INFO("Publishing planning arm states...");
        ros::Rate rate = 1.0; // HACK: make sure sbpl gets the new armstate
        rate.sleep();
    }
    else
    {
        ROS_ERROR("%s Could not switch to topic \"%s\".", __PRETTY_FUNCTION__, switchSrv.request.topic.c_str());
    }
}

void switchToExecutionTopic()
{
    if (!s_SwitchJointTopicClient)
    {
        ROS_ERROR("Persistent service connection to %s failed.", s_SwitchJointTopicClient.getService().c_str());
        return;
    }
    topic_tools::MuxSelect switchSrv;
    switchSrv.request.topic = "/joint_states_throttle";
    if (s_SwitchJointTopicClient.call(switchSrv))
    {
        ROS_INFO("%s: switched to topic \"%s\".", __FUNCTION__, switchSrv.request.topic.c_str());
    }
    else
    {
        ROS_ERROR("%s Could not switch to topic \"%s\".", __PRETTY_FUNCTION__, switchSrv.request.topic.c_str());
    }
}

void fullbody_navstack_init(int argc, char** argv)
{
    navstack_init(argc, argv);

    // init service query for joint topic switcher
    std::string service_name = "/mux_joint_states/select";
    while (!ros::service::waitForService(service_name, ros::Duration(3.0)))
    {
        ROS_ERROR("Service %s not available - waiting.", service_name.c_str());
    }
    s_SwitchJointTopicClient = g_NodeHandle->serviceClient<topic_tools::MuxSelect>(service_name, true);
    if (!s_SwitchJointTopicClient)
    {
        ROS_FATAL("Could not initialize get plan service from %s (client name: %s)", service_name.c_str(), s_SwitchJointTopicClient.getService().c_str());
    }

    // init joint state publisher
    receivedJointState = false;
    s_JointStateSubscriber = g_NodeHandle->subscribe("/joint_states_throttle", 3, jointStateCallback);
    s_PlanningJointStatePublisher = g_NodeHandle->advertise<sensor_msgs::JointState>("/joint_states_tfd", 5, false);

    ROS_INFO("Initialized full body navstack module.");
}

double fullbody_pathCost(const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
    nav_msgs::GetPlan srv;
    if (!fillPathRequest(parameterList, numericalFluentCallback, srv.request))
    {
        return INFINITE_COST;
    }

    // first lookup in the cache if we answered the query already
    double cost = 0;
    if (g_PathCostCache.get(computePathCacheKey(parameterList[0].value, parameterList[1].value, srv.request.start.pose, srv.request.goal.pose), cost))
    {
        return cost;
    }
    publishPlanningArmState();
    cost = pathCost(parameterList, predicateCallback, numericalFluentCallback, relaxed);
    switchToExecutionTopic();
    return cost;
}

