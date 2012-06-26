#include "tidyup_actions/actionExecutorPlaceObject.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_place_object,
        tidyup_actions::ActionExecutorPlaceObject,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{

    bool ActionExecutorPlaceObject::fillGoal(tidyup_msgs::PlaceObjectGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {

    }

    void ActionExecutorPlaceObject::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const tidyup_msgs::PlaceObjectResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("PlaceObject returned result");
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("PlaceObject succeeded.");

        }
    }

};

