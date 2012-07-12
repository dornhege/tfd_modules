#include "tidyup_actions/actionExecutorOpenDoor.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_open_door,
        tidyup_actions::ActionExecutorOpenDoor,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{

    bool ActionExecutorOpenDoor::fillGoal(tidyup_msgs::OpenDoorGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 3);
        string location = a.parameters[0];
        string door = a.parameters[1];
        string arm = a.parameters[2];

        // set arm
        goal.left_arm = false;
        goal.right_arm = false;
        if(arm == "left_arm") {
            goal.left_arm = true;
        }
        if(arm == "right_arm") {
            goal.right_arm = true;
        }
        return goal.right_arm || goal.left_arm;
    }

    void ActionExecutorOpenDoor::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const tidyup_msgs::OpenDoorResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("OpenDoor returned result");
        ROS_ASSERT(a.parameters.size() == 3);
        string location = a.parameters[0];
        string door = a.parameters[1];
        string arm = a.parameters[2];
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("OpenDoor succeeded.");
            current.setBooleanPredicate("door-open", door, true);
        }
        current.setObjectFluent("arm-state", arm, "arm_unknown");
        current.setBooleanPredicate("door-state-known", door, false);
    }

};

