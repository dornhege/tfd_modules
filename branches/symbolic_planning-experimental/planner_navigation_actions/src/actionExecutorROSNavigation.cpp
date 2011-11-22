#include "planner_navigation_actions/actionExecutorROSNavigation.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(planner_navigation_actions, action_executor_ros_navigation,
        planner_navigation_actions::ActionExecutorROSNavigation,
        continual_planning_executive::ActionExecutorInterface)

namespace planner_navigation_actions
{

    ActionExecutorROSNavigation::ActionExecutorROSNavigation()
    {
        _actionClient = new MoveBaseClient("move_base", true);
        // wait for the action server to come up
        while(!_actionClient->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
    }

    ActionExecutorROSNavigation::~ActionExecutorROSNavigation()
    {
        // TODO: stop and delete
    }

    bool ActionExecutorROSNavigation::canExecute(const DurativeAction & a, const SymbolicState & current) const
    {
        // TODO: check this is the right kind of action and we do not have one running

        return true;
    }

    bool ActionExecutorROSNavigation::executeBlocking(const DurativeAction & a, SymbolicState & current)
    {
        move_base_msgs::MoveBaseGoal goal;

        //we'll send a goal to the robot to move 1 meter forward
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

        ROS_INFO("Sending goal");
        _actionClient->sendGoal(goal);

        // blocking call
        _actionClient->waitForResult();

        if(_actionClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Reached move_base target.");
            current.setBooleanPredicate("explored", targetName, true);
            return true;
        }

        ROS_INFO("Could not reach target! State: %s.", _actionClient->getState().toString().c_str());
        return false;
    }

};

