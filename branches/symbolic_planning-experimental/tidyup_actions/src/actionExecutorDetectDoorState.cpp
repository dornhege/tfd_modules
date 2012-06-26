#include "tidyup_actions/actionExecutorDetectDoorState.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_detect_door_state,
        tidyup_actions::ActionExecutorDetectDoorState,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{

    bool ActionExecutorDetectDoorState::fillGoal(tidyup_msgs::DetectDoorState::Request & goal,
            const DurativeAction & a, const SymbolicState & current)
    {

    }

    void ActionExecutorDetectDoorState::updateState(bool success, tidyup_msgs::DetectDoorState::Response & response,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("DetectDoorState returned result");
        if(success) {
            ROS_INFO("DetectDoorState succeeded.");

        }
    }

};

