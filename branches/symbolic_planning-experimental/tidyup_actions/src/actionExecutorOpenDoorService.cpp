#include "tidyup_actions/actionExecutorOpenDoorService.h"
#include <pluginlib/class_list_macros.h>
#include "tidyup_utils/planning_scene_interface.h"

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_open_door_service,
        tidyup_actions::ActionExecutorOpenDoorService,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{

    bool ActionExecutorOpenDoorService::fillGoal(std_srvs::Empty::Request & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        if(!PlanningSceneInterface::instance()->resetPlanningScene())   // FIXME try anyways?
            ROS_ERROR("%s: PlanningScene reset failed.", __PRETTY_FUNCTION__);

        return true;
    }

    void ActionExecutorOpenDoorService::updateState(bool success,
            std_srvs::Empty::Response & response,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("OpenDoorService returned result");
        ROS_ASSERT(a.parameters.size() == 3);
        string location = a.parameters[0];
        string door = a.parameters[1];
        string arm = a.parameters[2];

        if(success) {
            ROS_INFO("OpenDoorService succeeded.");
            current.setBooleanPredicate("door-open", door, true);
        }

        current.setObjectFluent("arm-state", arm, "arm_unknown");
        current.setBooleanPredicate("door-state-known", door, false);
    }

};

