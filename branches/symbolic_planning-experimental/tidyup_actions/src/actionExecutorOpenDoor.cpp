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

    }

    void ActionExecutorOpenDoor::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const tidyup_msgs::OpenDoorResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("OpenDoor returned result");
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("OpenDoor succeeded.");

        }
    }

};

