#include "tidyup_fleximove_actions/actionExecutorArmToCarry.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_fleximove_actions, action_executor_post_grasp_position,
        tidyup_fleximove_actions::ActionExecutorArmToCarry,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_fleximove_actions
{

    bool ActionExecutorArmToCarry::fillGoal(tidyup_msgs::PostGraspPositionGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        goal.left_arm = false;
        goal.right_arm = false;

        ROS_ASSERT(a.parameters.size() == 1);
        string arm = a.parameters[0];
        if(arm == "left_arm") {
            goal.left_arm = true;
        }
        if(arm == "right_arm") {
            goal.right_arm = true;
        }

        return (goal.left_arm || goal.right_arm);
    }

    void ActionExecutorArmToCarry::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const tidyup_msgs::PostGraspPositionResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("PostGraspPosition returned result: %s", result.result.c_str());
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Post Grasp Position succeeded.");
            ROS_ASSERT(a.parameters.size() == 1);
            string arm = a.parameters[0];
            current.setObjectFluent("arm-state", arm, "arm_carrying");
        }
    }

};

