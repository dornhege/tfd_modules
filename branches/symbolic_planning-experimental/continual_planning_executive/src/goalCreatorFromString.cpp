#include <continual_planning_executive/goalCreatorFromString.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

PLUGINLIB_DECLARE_CLASS(continual_planning_executive, goal_creator_from_string,
        continual_planning_executive::GoalCreatorFromString, continual_planning_executive::GoalCreator)

namespace continual_planning_executive
{
    GoalCreatorFromString::GoalCreatorFromString()
    {
    }

    GoalCreatorFromString::~GoalCreatorFromString()
    {
    }

    void GoalCreatorFromString::initialize(const std::deque<std::string> & arguments)
    {
        ROS_ASSERT(arguments.size() == 1);

        goal_statement = arguments[0];
    }

    bool GoalCreatorFromString::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
        goal.setStringGoalStatement(goal_statement);
        return true;
    }

};

