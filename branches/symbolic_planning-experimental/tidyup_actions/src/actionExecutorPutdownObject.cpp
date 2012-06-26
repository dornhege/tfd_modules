#include "tidyup_actions/actionExecutorPutdownObject.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_putdown_object,
        tidyup_actions::ActionExecutorPutdownObject,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{

    bool ActionExecutorPutdownObject::fillGoal(tidyup_msgs::PlaceObjectGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 4);
        string location = a.parameters[0];
        string object = a.parameters[1];
        string static_object = a.parameters[2];
        string arm = a.parameters[3];

        // set arm
        goal.left_arm = false;
        goal.right_arm = false;
        if(arm == "left_arm") {
            goal.left_arm = true;
        }
        if(arm == "right_arm") {
            goal.right_arm = true;
        }

        goal.surface_name = static_object;

        return (goal.left_arm || goal.right_arm);
    }

    void ActionExecutorPutdownObject::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const tidyup_msgs::PlaceObjectResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("PutdownObject returned result");
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("PutdownObject succeeded.");
            ROS_ASSERT(a.parameters.size() == 4);
            string location = a.parameters[0];
            string object = a.parameters[1];
            string static_object = a.parameters[2];
            string arm = a.parameters[3];
            current.setBooleanPredicate("grasped", object + " " + arm, false);
            current.setBooleanPredicate("on", object + " " + static_object, true);
            current.setObjectFluent("arm-position", arm, "arm_unknown");
            current.setBooleanPredicate("graspable-from", object + " " + location + " " + arm, true);
            current.setBooleanPredicate("recent-detected-objects", location, false);
            current.setBooleanPredicate("searched", location, false);
            current.setNumericalFluent("x", object, result.position.pose.position.x);
            current.setNumericalFluent("y", object, result.position.pose.position.y);
            current.setNumericalFluent("z", object, result.position.pose.position.z);
            current.setNumericalFluent("qx", object, result.position.pose.orientation.x);
            current.setNumericalFluent("qy", object, result.position.pose.orientation.y);
            current.setNumericalFluent("qz", object, result.position.pose.orientation.z);
            current.setNumericalFluent("qw", object, result.position.pose.orientation.w);
            current.setNumericalFluent("timestamp", object, result.position.header.stamp.sec);
            current.setObjectFluent("frame-id", object, result.position.header.frame_id);
        }
    }
};

