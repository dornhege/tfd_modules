#include "planner_navigation_actions/actionExecutorROSNavigation.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(planner_navigation_actions, action_executor_ros_navigation,
        planner_navigation_actions::ActionExecutorROSNavigation,
        continual_planning_executive::ActionExecutorInterface)

namespace planner_navigation_actions
{

    void ActionExecutorROSNavigation::initialize(const std::deque<std::string> & arguments)
    {
        ActionExecutorActionlib<move_base_msgs::MoveBaseAction, move_base_msgs::MoveBaseGoal,
            move_base_msgs::MoveBaseResult>::initialize(arguments);

        if(arguments.size() < 3)
            return;

        bool parseStart = true;
        if(arguments[2] == "goal") {
            parseStart = false;
        } else {
            ROS_ASSERT(arguments[2] == "start");
        }

        unsigned int curArg = 3;
        while(curArg < arguments.size()) {
            if(arguments[curArg] == "goal") {
                parseStart = false;
                curArg++;
                continue;
            }
            ROS_ASSERT(arguments.size() >= curArg + 2);  // need to access curArg, curArg+1
            string pred = arguments[curArg];
            string setS = arguments[curArg + 1];
            bool set = false;
            if(setS == "true")
                set = true;
            else if(setS == "false")
                set = false;
            else
                ROS_ASSERT(false);
            if(parseStart)
                _startPredicates.push_back(std::make_pair(pred, set));
            else
                _goalPredicates.push_back(std::make_pair(pred, set));
            curArg += 2;
        }
    }

    bool ActionExecutorROSNavigation::fillGoal(move_base_msgs::MoveBaseGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        // FIXME: don't get from state (very old), but the newest.
        // The frame_id should be a fixed frame anyways
        goal.target_pose.header.stamp = ros::Time::now();

        ROS_ASSERT(a.parameters.size() == 2);
        string targetName = a.parameters[1];

        // extract nicer + warn.
        Predicate p;
        p.parameters.push_back(targetName);
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

    void ActionExecutorROSNavigation::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const move_base_msgs::MoveBaseResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_ASSERT(a.parameters.size() == 2);
            string startName = a.parameters[0];
            string targetName = a.parameters[1];
            for(std::vector<std::pair<std::string, bool> >::iterator it = _startPredicates.begin();
                    it != _startPredicates.end(); it++) {
                current.setBooleanPredicate(it->first, startName, it->second);
            }
            for(std::vector<std::pair<std::string, bool> >::iterator it = _goalPredicates.begin();
                    it != _goalPredicates.end(); it++) {
                current.setBooleanPredicate(it->first, targetName, it->second);
            }

        }
    }

};

