#include "tidyup_actions/actionExecutorPickupSponge.h"
#include <pluginlib/class_list_macros.h>
#include "tidyup_utils/planning_scene_interface.h"

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_pickup_sponge,
        tidyup_actions::ActionExecutorPickupSponge,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{

    bool ActionExecutorPickupSponge::fillGoal(std_srvs::Empty::Request & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        if(!PlanningSceneInterface::instance()->resetPlanningScene())   // FIXME try anyways?
            ROS_ERROR("%s: PlanningScene reset failed.", __PRETTY_FUNCTION__);

        return true;
    }

    void ActionExecutorPickupSponge::updateState(bool success,
            std_srvs::Empty::Response & response,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("PickupSponge returned result");
        ROS_ASSERT(a.parameters.size() == 1);
        string arm = a.parameters[0];

        if(success) {
            ROS_INFO("picking up sponge with %s succeeded.", arm.c_str());
            current.setBooleanPredicate("grasped-sponge", arm, true);
        } else {
            ROS_ERROR("picking up sponge with %s FAILED.", arm.c_str());
        }

        current.setObjectFluent("arm-state", arm, "arm_unknown");
    }

};

