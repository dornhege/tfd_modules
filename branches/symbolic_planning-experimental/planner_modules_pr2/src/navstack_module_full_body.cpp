#include "planner_modules_pr2/navstack_module_full_body.h"
#include "planner_modules_pr2/navstack_module.h"
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

ros::ServiceClient s_SwitchJointTopicClient;
ros::Subscriber s_JointStateSubscriber;
ros::Publisher s_PlanningJointStatePublisher;
sensor_msgs::JointState s_CurrentState;
sensor_msgs::JointState s_RightArmAtSide;
sensor_msgs::JointState s_LeftArmAtSide;
bool receivedJointState;

void jointStateCallback(const sensor_msgs::JointState& msg)
{
    receivedJointState = true;
    s_CurrentState = msg;
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
        replaceJointPosition(s_CurrentState, s_RightArmAtSide);
        replaceJointPosition(s_CurrentState, s_LeftArmAtSide);
        s_PlanningJointStatePublisher.publish(s_CurrentState);
        ROS_INFO("Publishing planning arm states...");
        ros::Rate rate = 1.0; // FIXME: HACK: make sure sbpl gets the new armstate
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

    // init joint state messages for possible arm states
    // right arm at side
    s_RightArmAtSide.name.push_back("r_shoulder_pan_joint");
    s_RightArmAtSide.name.push_back("r_shoulder_lift_joint");
    s_RightArmAtSide.name.push_back("r_upper_arm_roll_joint");
    s_RightArmAtSide.name.push_back("r_elbow_flex_joint");
    s_RightArmAtSide.name.push_back("r_forearm_roll_joint");
    s_RightArmAtSide.name.push_back("r_wrist_flex_joint");
    s_RightArmAtSide.name.push_back("r_wrist_roll_joint");
    if (g_NodeHandle->hasParam("/arm_configurations/side_tuck/position/right_arm"))
    {
        XmlRpc::XmlRpcValue paramList;
        g_NodeHandle->getParam("/arm_configurations/side_tuck/position/right_arm", paramList);
        ROS_ASSERT(paramList.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < paramList.size(); ++i)
        {
          ROS_ASSERT(paramList[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          s_RightArmAtSide.position.push_back(static_cast<double>(paramList[i]));
        }
    }
    else
    {
        // DEFAULT [-2.110, 1.230, -2.06, -1.69, 3.439, -1.52, 1.57]
        s_RightArmAtSide.position.push_back(-2.110);
        s_RightArmAtSide.position.push_back(1.230);
        s_RightArmAtSide.position.push_back(-2.06);
        s_RightArmAtSide.position.push_back(-1.69);
        s_RightArmAtSide.position.push_back(3.439);
        s_RightArmAtSide.position.push_back(-1.52);
        s_RightArmAtSide.position.push_back(1.57);
    }
    // left arm at side
    s_LeftArmAtSide.name.push_back("l_shoulder_pan_joint");
    s_LeftArmAtSide.name.push_back("l_shoulder_lift_joint");
    s_LeftArmAtSide.name.push_back("l_upper_arm_roll_joint");
    s_LeftArmAtSide.name.push_back("l_elbow_flex_joint");
    s_LeftArmAtSide.name.push_back("l_forearm_roll_joint");
    s_LeftArmAtSide.name.push_back("l_wrist_flex_joint");
    s_LeftArmAtSide.name.push_back("l_wrist_roll_joint");
    if (g_NodeHandle->hasParam("/arm_configurations/side_tuck/position/left_arm"))
    {
        XmlRpc::XmlRpcValue paramList;
        g_NodeHandle->getParam("/arm_configurations/side_tuck/position/left_arm", paramList);
        ROS_ASSERT(paramList.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < paramList.size(); ++i)
        {
          ROS_ASSERT(paramList[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          s_LeftArmAtSide.position.push_back(static_cast<double>(paramList[i]));
        }
    }
    else
    {
        // DEFAULT [2.110, 1.230, 2.06, -1.69, -3.439, -1.52, 1.57]
        s_LeftArmAtSide.position.push_back(2.110);
        s_LeftArmAtSide.position.push_back(1.230);
        s_LeftArmAtSide.position.push_back(2.06);
        s_LeftArmAtSide.position.push_back(-1.69);
        s_LeftArmAtSide.position.push_back(-3.439);
        s_LeftArmAtSide.position.push_back(-1.52);
        s_LeftArmAtSide.position.push_back(1.57);
    }

    ROS_INFO("Initialized full body navstack module.\n");
}

double fullbody_pathCost(const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
    // first lookup in the cache if we answered the query already
    map<pair<string, string>, double>::iterator it = g_PathCostCache.find(make_pair(parameterList[0].value, parameterList[1].value));
    if (it != g_PathCostCache.end())
    {
        return it->second;
    }
    publishPlanningArmState();
    double cost = pathCost(parameterList, predicateCallback, numericalFluentCallback, relaxed);
    switchToExecutionTopic();
    return cost;
}

