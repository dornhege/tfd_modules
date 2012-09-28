#include "tidyup_actions/actionExecutorWipe.h"
#include <pluginlib/class_list_macros.h>
#include "tidyup_utils/planning_scene_interface.h"

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_wipe,
        tidyup_actions::ActionExecutorWipe,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{

    void ActionExecutorWipe::initialize(const std::deque<std::string> & arguments)
    {
        ActionExecutorService<coverage_srvs::CleanSpot>::initialize(arguments);
    }

    bool ActionExecutorWipe::fillGoal(coverage_srvs::CleanSpot::Request & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        if(!PlanningSceneInterface::instance()->resetPlanningScene())   // FIXME try anyways?
            ROS_ERROR("%s: PlanningScene reset failed.", __PRETTY_FUNCTION__);

        goal.box_size = 0.3;

        // get spot from state
        ROS_ASSERT(a.parameters.size() == 4);
        string location = a.parameters[0];
        string wipe_point = a.parameters[1];
        string static_object = a.parameters[2];
        string arm = a.parameters[3];

        Predicate p;
        p.parameters.push_back(wipe_point);
        p.name = "frame-id";
        if(!current.hasObjectFluent(p, &goal.spot.header.frame_id))
            return false;
        p.name = "x";
        if(!current.hasNumericalFluent(p, &goal.spot.point.x))
            return false;
        p.name = "y";
        if(!current.hasNumericalFluent(p, &goal.spot.point.y))
            return false;
        p.name = "z";
        if(!current.hasNumericalFluent(p, &goal.spot.point.z))
            return false;

        ROS_INFO_STREAM("Created goal for ActionExecutorWipe at: " << goal.spot);

        return true;
    }

    void ActionExecutorWipe::updateState(bool success, coverage_srvs::CleanSpot::Response & response,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("Wipe returned result");

        ROS_ASSERT(a.parameters.size() == 4);
        string location = a.parameters[0];
        string wipe_point = a.parameters[1];
        string static_object = a.parameters[2];
        string arm = a.parameters[3];

        if(success) {
            ROS_INFO("Wipe succeeded.");
            current.setBooleanPredicate("wiped", wipe_point, true);
        } else {
            ROS_ERROR("Wiping failed.");
        }
        current.setObjectFluent("arm-state", arm, "arm_unknown");
    }

};

