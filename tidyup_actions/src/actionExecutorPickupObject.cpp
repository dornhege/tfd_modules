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
        goal.pickup_object = a.parameters[1];
        goal.static_object = a.parameters[2];
        goal.arm = a.parameters[3];

        return true;
    }

    void ActionExecutorPickupObject::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const tidyup_msgs::GraspObjectResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("PickupObject returned result");
        ROS_ASSERT(a.parameters.size() == 4);
        string location = a.parameters[0];
        string object = a.parameters[1];
        string static_object = a.parameters[2];
        string arm = a.parameters[3];
// TODO: set grasp in symbolic state?       result.grasp;
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("PickupObject succeeded.");
            current.setBooleanPredicate("searched", location, false);
            current.setBooleanPredicate("grasped", object + " " + arm, true);
            current.setBooleanPredicate("on", object + " " + static_object, false);
            current.setBooleanPredicate("graspable-from", object + " " + location + " left_arm", false);
            current.setBooleanPredicate("graspable-from", object + " " + location + " right_arm", false);
            // TODO: set false for other locations as well?
        }
        current.setObjectFluent("arm-state", arm, "arm_unknown");
    }

};

