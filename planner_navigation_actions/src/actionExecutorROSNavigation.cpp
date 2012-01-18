#include "planner_navigation_actions/actionExecutorROSNavigation.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(planner_navigation_actions, action_executor_ros_navigation,
        planner_navigation_actions::ActionExecutorROSNavigation,
        continual_planning_executive::ActionExecutorInterface)

namespace planner_navigation_actions
{

    bool ActionExecutorROSNavigation::fillGoal(move_base_msgs::MoveBaseGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        goal.target_pose.header.frame_id = "/map";      // TODO: whole thing tf frame resolv?
        goal.target_pose.header.stamp = ros::Time::now();

        ROS_ASSERT(a.parameters.size() == 2);
        string targetName = a.parameters[1];

        // extract nicer + warn.
        Predicate p;
        p.parameters.push_back(targetName);
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
        return true;
    }

    void ActionExecutorROSNavigation::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const move_base_msgs::MoveBaseResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_ASSERT(a.parameters.size() == 2);
            string targetName = a.parameters[1];
            current.setBooleanPredicate("explored", targetName, true);
            // TODO fixme generic
            current.setAllBooleanPredicates("recent-detected-objects", false);
        }
    }

};

