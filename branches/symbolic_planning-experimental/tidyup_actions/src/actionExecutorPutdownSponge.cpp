#include "tidyup_actions/actionExecutorPutdownSponge.h"
#include <pluginlib/class_list_macros.h>
#include "tidyup_utils/planning_scene_interface.h"

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_putdown_sponge,
        tidyup_actions::ActionExecutorPutdownSponge,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{

    bool ActionExecutorPutdownSponge::fillGoal(std_srvs::Empty::Request & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        if(!PlanningSceneInterface::instance()->resetPlanningScene())   // FIXME try anyways?
            ROS_ERROR("%s: PlanningScene reset failed.", __PRETTY_FUNCTION__);

        return true;
    }

    void ActionExecutorPutdownSponge::updateState(bool success,
            std_srvs::Empty::Response & response,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("PutdownSponge returned result");
        ROS_ASSERT(a.parameters.size() == 1);
        string arm = a.parameters[0];

        if(success) {
            ROS_INFO("PutdownSponge succeeded.");
            current.setBooleanPredicate("grasped-sponge", arm, false);
        } else {
            ROS_ERROR("PutdownSponge failed.");
        }

        current.setObjectFluent("arm-state", arm, "arm_unknown");
    }

};

