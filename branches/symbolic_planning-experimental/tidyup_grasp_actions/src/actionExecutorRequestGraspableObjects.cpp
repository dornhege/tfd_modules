#include "tidyup_grasp_actions/actionExecutorRequestGraspableObjects.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_grasp_actions, action_executor_request_graspable_objects,
        tidyup_grasp_actions::ActionExecutorRequestGraspableObjects,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_grasp_actions
{

    bool ActionExecutorRequestGraspableObjects::fillGoal(tidyup_msgs::RequestGraspableObjects::Request & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        goal.onlyReachable = true;
        return true;
    }

    void ActionExecutorRequestGraspableObjects::updateState(bool success,
            tidyup_msgs::RequestGraspableObjects::Response & response,
            const DurativeAction & a, SymbolicState & current)
    {
        if(!success)
            return;

        ROS_ASSERT(a.parameters.size() == 1);
        std::string location = a.parameters.at(0);

        for(std::vector<tidyup_msgs::GraspableObject>::iterator it = response.objects.begin();
                it != response.objects.end(); it++) {
            tidyup_msgs::GraspableObject & object = *it;

            current.addObject(object.name, "movable_object");
            current.addObject(object.pose.header.frame_id, "frameid");
            current.setBooleanPredicate("graspable-from", object.name + " " + location, true);
            current.setNumericalFluent("x", object.name, object.pose.pose.position.x);
            current.setNumericalFluent("y", object.name, object.pose.pose.position.y);
            current.setNumericalFluent("z", object.name, object.pose.pose.position.z);
            current.setNumericalFluent("qx", object.name, object.pose.pose.orientation.x);
            current.setNumericalFluent("qy", object.name, object.pose.pose.orientation.y);
            current.setNumericalFluent("qz", object.name, object.pose.pose.orientation.z);
            current.setNumericalFluent("qw", object.name, object.pose.pose.orientation.w);
            current.setNumericalFluent("timestamp", object.name, object.pose.header.stamp.toSec());
            current.setObjectFluent("frame-id", object.name, object.pose.header.frame_id);
            ROS_ASSERT(object.reachable);
        }

        current.setBooleanPredicate("detected-objects", location, true);
        current.setBooleanPredicate("recent-detected-objects", location, true);
    }

};

