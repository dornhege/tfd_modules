#include "tidyup_actions/actionExecutorDetectDoorStateAngle.h"
#include <pluginlib/class_list_macros.h>
#include "tidyup_utils/planning_scene_interface.h"

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_detect_door_state_angle,
        tidyup_actions::ActionExecutorDetectDoorStateAngle,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{

    void ActionExecutorDetectDoorStateAngle::initialize(const std::deque<std::string> & arguments)
    {
        ActionExecutorService<tidyup_msgs::DoorState>::initialize(arguments);
    }

    bool ActionExecutorDetectDoorStateAngle::fillGoal(tidyup_msgs::DoorState::Request & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        if(!PlanningSceneInterface::instance()->resetPlanningScene())   // FIXME try anyways?
            ROS_ERROR("%s: PlanningScene reset failed.", __PRETTY_FUNCTION__);

        return true;
    }

    void ActionExecutorDetectDoorStateAngle::updateState(bool success, tidyup_msgs::DoorState::Response & response,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("DetectDoorStateAngle returned result");
        if(success) {
            ROS_INFO("DetectDoorStateAngle succeeded, door_found is: %d, angle is: %f.", response.door_found, response.angle);
            ROS_ASSERT(a.parameters.size() == 2);
            string location = a.parameters[0];
            string door = a.parameters[1];
            bool open = false;
            if(!response.door_found) {
                open = true;
                ROS_INFO("No door found - assuming it is open.");
            } else {
                if(response.angle >= response.DOOR_OPEN_ANGLE) {
                    open = true;
                    ROS_INFO("Angle was %f > %f - Door is open.", response.angle, response.DOOR_OPEN_ANGLE);
                } else if(response.angle <= response.DOOR_CLOSED_ANGLE) {
                    open = false;
                    ROS_INFO("Angle was %f < %f - Door is closed.", response.angle, response.DOOR_CLOSED_ANGLE);
                } else {
                    ROS_ERROR("Angle was %f - Unidentified door state - assuming it is closed!", response.angle);
                    open = false;
                }
            }
            current.setBooleanPredicate("door-state-known", door, true);
            current.setBooleanPredicate("door-open", door, open);
        }
    }

};

