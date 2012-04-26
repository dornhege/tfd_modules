#include "tidyup_place_actions/actionExecutorArmToSide.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_place_actions, action_executor_arm_to_side,
        tidyup_place_actions::ActionExecutorArmToSide,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_place_actions
{

    bool ActionExecutorArmToSide::fillGoal(pr2_python_services::ArmToSideGoal & goal,
                        const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 1);
        goal.arm = a.parameters[0];
        if(goal.arm == "left_arm") {
            return true;
        }
        if(goal.arm == "right_arm") {
            return true;
        }

        return false;
    }

    void ActionExecutorArmToSide::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
    		const pr2_python_services::ArmToSideResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("ArmToSide returned result: %s", result.result.c_str());
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Arm Side Position succeeded.");
            ROS_ASSERT(a.parameters.size() == 1);
            string arm = a.parameters[0];
            current.setObjectFluent("arm-position", arm, "arm_at_side");
        }
    }

};

