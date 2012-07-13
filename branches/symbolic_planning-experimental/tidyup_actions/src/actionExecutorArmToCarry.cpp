#include "tidyup_actions/actionExecutorArmToCarry.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_arm_to_carry,
        tidyup_actions::ActionExecutorArmToCarry,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{

    void ActionExecutorArmToCarry::initialize(const std::deque<std::string> & arguments)
    {
        ActionExecutorActionlib<tidyup_msgs::PostGraspPositionAction, tidyup_msgs::PostGraspPositionGoal,
        tidyup_msgs::PostGraspPositionResult>::initialize(arguments);

        _armStatePredicateName = "arm-state";
        _armAtCarryConstantName = "arm_carrying";

        if(arguments.size() >= 3) {     // 3rd param: arm-state predicate name
            _armStatePredicateName = arguments[2];
        }
        if(arguments.size() >= 4) {     // 4th param: arm_at_side constant name
            _armAtCarryConstantName = arguments[3];
        }   
    }               

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
        ROS_ASSERT(a.parameters.size() == 1);
        string arm = a.parameters[0];
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Post Grasp Position succeeded.");
            current.setObjectFluent(_armStatePredicateName, arm, _armAtCarryConstantName);
        } else {
            current.setObjectFluent(_armStatePredicateName, arm, "arm_unknown");
        }
    }

};

