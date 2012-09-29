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
            ROS_INFO("putting down sponge with %s succeeded.", arm.c_str());
            current.setBooleanPredicate("grasped-sponge", arm, false);

            // tell world that we putdown the sponge
            const char* DETACH_SERVICE = "/tidyup/detach_sponge";
            if(!ros::service::exists(DETACH_SERVICE, true)) {
                ROS_ERROR("%s: Service %s does not exist.", __func__, DETACH_SERVICE);
            }
            std_srvs::Empty srv;
            if(!ros::service::call(DETACH_SERVICE, srv)) {
                ROS_ERROR("%s: Service call to %s failed.", __func__, DETACH_SERVICE);
            }
        } else {
            ROS_ERROR("putting down sponge with %s FAILED.", arm.c_str());
        }

        current.setObjectFluent("arm-state", arm, "arm_unknown");
    }

};

