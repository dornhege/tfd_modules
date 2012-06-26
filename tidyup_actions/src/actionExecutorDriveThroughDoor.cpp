#include "tidyup_actions/actionExecutorDriveThroughDoor.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_move_base,
        tidyup_actions::ActionExecutorDriveThroughDoor,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{

    bool ActionExecutorDriveThroughDoor::fillGoal(move_base_msgs::MoveBaseGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        goal.target_pose.header.stamp = ros::Time::now();

        ROS_ASSERT(a.parameters.size() == 3);
        string door = a.parameters[0];
        string start_location = a.parameters[1];
        string goal_location = a.parameters[2];

        // extract nicer + warn.
        Predicate p;
        p.parameters.push_back(goal_location);
        p.name = "frame-id";
        if(!current.hasObjectFluent(p, &goal.target_pose.header.frame_id))
            return false;
        p.name = "x";
        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.position.x))
            return false;
        p.name = "y";
        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.position.y))
            return false;
        p.name = "z";
        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.position.z))
            return false;
        p.name = "qx";
        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.orientation.x))
            return false;
        p.name = "qy";
        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.orientation.y))
            return false;
        p.name = "qz";
        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.orientation.z))
            return false;
        p.name = "qw";
        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.orientation.w))
            return false;

        ROS_INFO_STREAM("Created goal for ActionExecutorROSNavigation as: " << goal);

        return true;
    }

    void ActionExecutorDriveThroughDoor::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const move_base_msgs::MoveBaseResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("MoveBase returned result");
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("MoveBase succeeded.");
            ROS_ASSERT(a.parameters.size() == 3);
            string door = a.parameters[0];
            string start_location = a.parameters[1];
            string goal_location = a.parameters[2];
            current.setBooleanPredicate("base-at", start_location, false);
            current.setBooleanPredicate("base-at", goal_location, true);
            current.setBooleanPredicate("door-state-known", door, false);
            string goal_room = "unknown_room";
            Predicate p;
            p.name = "location-in-room";
            p.parameters.push_back(goal_location);
            if(current.hasObjectFluent(p, &goal_room))
                current.setObjectFluent("location-in-room", "robot_location", goal_room);
        }
    }

};

