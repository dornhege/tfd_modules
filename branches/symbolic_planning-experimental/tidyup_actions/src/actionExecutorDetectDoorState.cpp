#include "tidyup_actions/actionExecutorDetectDoorState.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_detect_door_state,
        tidyup_actions::ActionExecutorDetectDoorState,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{

    void ActionExecutorDetectDoorState::initialize(const std::deque<std::string> & arguments)
    {
        ActionExecutorService<tidyup_msgs::DetectDoorState>::initialize(arguments);
    }

    bool ActionExecutorDetectDoorState::fillGoal(tidyup_msgs::DetectDoorState::Request & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        return true;
    }

    void ActionExecutorDetectDoorState::updateState(bool success, tidyup_msgs::DetectDoorState::Response & response,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("DetectDoorState returned result");
        if(success) {
            ROS_INFO("DetectDoorState succeeded.");
            ROS_ASSERT(a.parameters.size() == 2);
            string location = a.parameters[0];
            string door = a.parameters[1];
            current.setBooleanPredicate("door-state-known", door, true);
            current.setBooleanPredicate("door-open", door, response.door_open);
        }
    }

};

