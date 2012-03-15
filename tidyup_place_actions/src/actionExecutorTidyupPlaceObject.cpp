#include "tidyup_place_actions/actionExecutorTidyupPlaceObject.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_place_actions, action_executor_place_object,
        tidyup_place_actions::ActionExecutorTidyupPlaceObject,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_place_actions
{

    bool ActionExecutorTidyupPlaceObject::fillGoal(tidyup_msgs::GraspObjectGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 4);
        string location = a.parameters[0];
        string targetName = a.parameters[1];
        string objectPose = a.parameters[2];
        string arm = a.parameters[3];

        // set arm
        goal.left_arm = false;
        goal.right_arm = false;
        if(arm == "left_arm") {
            goal.left_arm = true;
        }
        if(arm == "right_arm") {
            goal.right_arm = true;
        }

        // set target
        goal.target.name = targetName;
        goal.target.reachable_left_arm = false;
        goal.target.reachable_right_arm = false;

        // set the target pose from state
        Predicate p;
        p.parameters.push_back(targetName);
        p.parameters.push_back(objectPose);

        p.name = "frame-id";
        if(!current.hasObjectFluent(p, &goal.target.pose.header.frame_id))
            return false;
        p.name = "timestamp";
        double ts;
        if(!current.hasNumericalFluent(p, &ts))
            return false;
        goal.target.pose.header.stamp = ros::Time(ts);
        p.name = "x";
        if(!current.hasNumericalFluent(p, &goal.target.pose.pose.position.x))
            return false;
        p.name = "y";
        if(!current.hasNumericalFluent(p, &goal.target.pose.pose.position.y))
            return false;
        p.name = "z";
        if(!current.hasNumericalFluent(p, &goal.target.pose.pose.position.z))
            return false;
        p.name = "qx";
        if(!current.hasNumericalFluent(p, &goal.target.pose.pose.orientation.x))
            return false;
        p.name = "qy";
        if(!current.hasNumericalFluent(p, &goal.target.pose.pose.orientation.y))
            return false;
        p.name = "qz";
        if(!current.hasNumericalFluent(p, &goal.target.pose.pose.orientation.z))
            return false;
        p.name = "qw";
        if(!current.hasNumericalFluent(p, &goal.target.pose.pose.orientation.w))
            return false;

        // set target reachable from state
        p.name = "can-putdown";
        p.parameters.push_back(location);

        p.parameters.push_back("left_arm");
        bool reachableLeft;
        if(!current.hasBooleanPredicate(p, &reachableLeft))
            return false;
        goal.target.reachable_left_arm = reachableLeft;

        p.parameters[3] = "right_arm";
        bool reachableRight;
        if(!current.hasBooleanPredicate(p, &reachableRight))
            return false;
        goal.target.reachable_right_arm = reachableRight;

        return (goal.left_arm || goal.right_arm);
    }

    void ActionExecutorTidyupPlaceObject::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const tidyup_msgs::GraspObjectResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("PlaceObject returned result: %s", result.result.c_str());
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Putdown succeeded.");
            ROS_ASSERT(a.parameters.size() == 4);
            string targetName = a.parameters[1];
            string objectPose = a.parameters[2];
            string arm = a.parameters[3];
            current.setObjectFluent("arm-position", arm, "unknown_armpos");
            current.setObjectFluent("at-object", targetName, objectPose);
            current.setBooleanPredicate("handFree", arm, true);
            current.setBooleanPredicate("grasped", targetName + " " + arm, false);
        }
    }

};

