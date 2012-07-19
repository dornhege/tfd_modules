/*
 * arm_state.cpp
 *
 *  Created on: 17 Jul 2012
 *      Author: andreas
 */

#include "planner_modules_pr2/arm_state.h"
#include <string>

using std::string;

string ArmState::armDescriptionBaseParameter = "/hand_description/";
string ArmState::armDescriptionArmLinksParameter = "/arm_joints/";

ArmState::ArmState(const string& armName, const string& armStateParameter)
: armName(armName)
{
    ros::NodeHandle g_NodeHandle = ros::NodeHandle();

    // load joint names from param server
    string linkNames = ArmState::armDescriptionBaseParameter+armName+ArmState::armDescriptionArmLinksParameter;
    if (g_NodeHandle.hasParam(linkNames))
    {
        XmlRpc::XmlRpcValue paramList;
        g_NodeHandle.getParam(linkNames, paramList);
        ROS_ASSERT(paramList.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < paramList.size(); ++i)
        {
            ROS_ASSERT(paramList[i].getType() == XmlRpc::XmlRpcValue::TypeString);
            armState.name.push_back(static_cast<string>(paramList[i]));
        }
    }
    else
    {
        string prefix = armName.substr(0, 1);
        armState.name.push_back(prefix+"_shoulder_pan_joint");
        armState.name.push_back(prefix+"_shoulder_lift_joint");
        armState.name.push_back(prefix+"_upper_arm_roll_joint");
        armState.name.push_back(prefix+"_elbow_flex_joint");
        armState.name.push_back(prefix+"_forearm_roll_joint");
        armState.name.push_back(prefix+"_wrist_flex_joint");
        armState.name.push_back(prefix+"_wrist_roll_joint");
    }

    // load joint positions from param server
    ROS_ASSERT(g_NodeHandle.hasParam(armStateParameter));
    XmlRpc::XmlRpcValue paramList;
    g_NodeHandle.getParam(armStateParameter, paramList);
    ROS_ASSERT(paramList.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < paramList.size(); ++i)
    {
        ROS_ASSERT(paramList[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        armState.position.push_back(static_cast<double>(paramList[i]));
    }
}

void ArmState::replaceJointPositions(sensor_msgs::JointState& state) const
{
    ArmState::replaceJointPositions(state, armState);
}

void ArmState::replaceJointPositions(sensor_msgs::JointState& state, const sensor_msgs::JointState& joints)
{
    for (unsigned int i = 0; i < joints.name.size(); i++)
    {
        string name = joints.name[i];
        int index = -1;
        for (unsigned int j = 0; j < state.name.size(); j++)
        {
            if (name.compare(state.name[j]) == 0)
            {
                index = j;
            }
        }
        if (index != -1)
        {
            state.position[index] = joints.position[i];
        }
    }
}




