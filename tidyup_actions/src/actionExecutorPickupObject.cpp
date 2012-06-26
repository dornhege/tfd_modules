#include "tidyup_actions/actionExecutorPickupObject.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_pickup_object,
        tidyup_actions::ActionExecutorPickupObject,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{

    bool ActionExecutorPickupObject::fillGoal(tidyup_msgs::GraspObjectGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 4);
        string location = a.parameters[0];
        string object = a.parameters[1];
        string static_object = a.parameters[2];
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
        goal.target.name = object;
        goal.target.reachable_left_arm = false;
        goal.target.reachable_right_arm = false;

        // set the target pose from state
        Predicate p;
        p.name = "at-object";
        p.parameters.push_back(object);
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
        p.name = "graspable-from";
        p.parameters.push_back(location);

        p.parameters.push_back("left_arm");
        bool reachableLeft = false;
        current.hasBooleanPredicate(p, &reachableLeft);
        goal.target.reachable_left_arm = reachableLeft;
        p.parameters[2] = "right_arm";
        bool reachableRight = false;
        current.hasBooleanPredicate(p, &reachableRight);
        goal.target.reachable_right_arm = reachableRight;

        return (goal.left_arm || goal.right_arm);
    }

    void ActionExecutorPickupObject::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const tidyup_msgs::GraspObjectResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("PickupObject returned result");
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("PickupObject succeeded.");
            ROS_ASSERT(a.parameters.size() == 4);
            string location = a.parameters[0];
            string object = a.parameters[1];
            string static_object = a.parameters[2];
            string arm = a.parameters[3];
            current.setObjectFluent("arm-state", arm, "arm_unknown");
            current.setBooleanPredicate("searched", location, false);
            current.setBooleanPredicate("grasped", object + " " + arm, true);
            current.setBooleanPredicate("on", object + " " + static_object, false);
            current.setBooleanPredicate("graspable-from", object + " " + location + " left_arm", false);
            current.setBooleanPredicate("graspable-from", object + " " + location + " right_arm", false);
            // TODO: set false for other locations as well?
        }
    }

};

