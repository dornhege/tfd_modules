#include "tidyup_actions/actionExecutorPutdownObject.h"
#include <pluginlib/class_list_macros.h>
#include "tidyup_msgs/GetPutdownPose.h"
#include "arm_navigation_msgs/ArmNavigationErrorCodes.h"
#include <arm_navigation_msgs/convert_messages.h>
#include "tidyup_utils/planning_scene_interface.h"

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_putdown_object,
        tidyup_actions::ActionExecutorPutdownObject,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{
    bool ActionExecutorPutdownObject::fillGoal(tidyup_msgs::PlaceObjectGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        if(!PlanningSceneInterface::instance()->resetPlanningScene())   // FIXME try anyways?
            ROS_ERROR("%s: PlanningScene reset failed.", __PRETTY_FUNCTION__);

        ROS_ASSERT(a.parameters.size() == 4);
        string location = a.parameters[0];
        goal.putdown_object = a.parameters[1];
        goal.static_object = a.parameters[2];
        goal.arm = a.parameters[3];
        return true;
    }

    void ActionExecutorPutdownObject::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const tidyup_msgs::PlaceObjectResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("PutdownObject returned result");
        ROS_ASSERT(a.parameters.size() == 4);
        string location = a.parameters[0];
        string object = a.parameters[1];
        string static_object = a.parameters[2];
        string arm = a.parameters[3];
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("PutdownObject succeeded.");
            current.setBooleanPredicate("grasped", object + " " + arm, false);
            current.setBooleanPredicate("on", object + " " + static_object, true);
            current.setBooleanPredicate("graspable-from", object + " " + location + " " + arm, true);
            current.setBooleanPredicate("searched", location, false);
            current.setNumericalFluent("x", object, result.putdown_pose.pose.position.x);
            current.setNumericalFluent("y", object, result.putdown_pose.pose.position.y);
            current.setNumericalFluent("z", object, result.putdown_pose.pose.position.z);
            current.setNumericalFluent("qx", object, result.putdown_pose.pose.orientation.x);
            current.setNumericalFluent("qy", object, result.putdown_pose.pose.orientation.y);
            current.setNumericalFluent("qz", object, result.putdown_pose.pose.orientation.z);
            current.setNumericalFluent("qw", object, result.putdown_pose.pose.orientation.w);
            current.setNumericalFluent("timestamp", object, result.putdown_pose.header.stamp.sec);
            current.setObjectFluent("frame-id", object, result.putdown_pose.header.frame_id);
            //current.setObjectFluent("object-detected-from", object, location); // FIXME hack, reset this
        }
        current.setObjectFluent("arm-state", arm, "arm_unknown");
        current.setBooleanPredicate("recent-detected-objects", location, false);
    }
};

