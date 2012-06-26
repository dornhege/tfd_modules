#include "tidyup_actions/actionExecutorArmToSide.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_arm_to_side,
        tidyup_actions::ActionExecutorArmToSide,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{

    void ActionExecutorArmToSide::initialize(const std::deque<std::string> & arguments)
    {
        ActionExecutorActionlib<tidyup_msgs::ArmToSideAction, tidyup_msgs::ArmToSideGoal,
            tidyup_msgs::ArmToSideResult>::initialize(arguments);

        _armStatePredicateName = "arm-state";
        _armAtSideConstantName = "arm_at_side";

        if(arguments.size() >= 3) {     // 3rd param: arm-state predicate name
            _armStatePredicateName = arguments[2];
        }
        if(arguments.size() >= 4) {     // 4th param: arm_at_side constant name
            _armAtSideConstantName = arguments[3];
        }
    }

    bool ActionExecutorArmToSide::fillGoal(tidyup_msgs::ArmToSideGoal & goal,
                        const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 1);
        if(a.parameters[0] == "left_arm") {
            goal.left_arm = true;
        }
        if(a.parameters[0] == "right_arm") {
            goal.right_arm = true;
        }

        return true;
    }

    void ActionExecutorArmToSide::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
    		const tidyup_msgs::ArmToSideResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("ArmToSide returned result: %s", result.result.c_str());
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Arm Side Position succeeded.");
            ROS_ASSERT(a.parameters.size() == 1);
            string arm = a.parameters[0];
            current.setObjectFluent(_armStatePredicateName, arm, _armAtSideConstantName);
        }
    }

};

