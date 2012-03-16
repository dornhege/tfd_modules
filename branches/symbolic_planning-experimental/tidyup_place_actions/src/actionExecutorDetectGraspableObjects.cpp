#include "tidyup_place_actions/actionExecutorDetectGraspableObjects.h"
#include <pluginlib/class_list_macros.h>
#include <tidyup_msgs/RequestObjectsGraspability.h>

PLUGINLIB_DECLARE_CLASS(tidyup_place_actions, action_executor_detect_graspable_objects,
        tidyup_place_actions::ActionExecutorDetectGraspableObjects,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_place_actions
{
    void ActionExecutorDetectGraspableObjects::initialize(const std::deque<std::string> & arguments)
    {
        ActionExecutorService<tidyup_msgs::DetectGraspableObjects>::initialize(arguments);

        ROS_ASSERT(_nh);

        if(s_RequestGraspability) {
            _serviceClientGraspability = _nh->serviceClient<tidyup_msgs::RequestObjectsGraspability>(
                    "/learned_grasping/request_objects_graspability");
            if(!_serviceClientGraspability) {
                ROS_FATAL("Could not initialize service for RequestObjectsGraspability.");
            }
        }
    }

    bool ActionExecutorDetectGraspableObjects::fillGoal(tidyup_msgs::DetectGraspableObjects::Request & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_WARN("fillGoal");
        return true;
    }

    void ActionExecutorDetectGraspableObjects::updateState(bool success,
            tidyup_msgs::DetectGraspableObjects::Response & response,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_WARN("updateState: ");
        if(!success)
            return;

        ROS_ASSERT(a.parameters.size() == 1);
        std::string location = a.parameters.at(0);

        std::vector<tidyup_msgs::GraspableObject> objects = response.objects;
        ROS_WARN("updateState: got %d objects", objects.size());

        if(s_RequestGraspability) {
            tidyup_msgs::RequestObjectsGraspability reqSrv;
            reqSrv.request.objects = objects;
            if(!_serviceClientGraspability.call(reqSrv)) {
                ROS_ERROR("Failed to call RequestObjectsGraspability service.");
            } else {
                objects = reqSrv.response.objects;
            }
        }

        for(std::vector<tidyup_msgs::GraspableObject>::iterator it = objects.begin();
                it != objects.end(); it++) {
            tidyup_msgs::GraspableObject & object = *it;

            current.addObject(object.name, "movable_object");
            if(object.pose.header.frame_id.empty()) {
                ROS_ERROR("DetectGraspableObjects returned empty frame_id for object: %s", object.name.c_str());
                object.pose.header.frame_id = "INVALID_FRAME_ID";
            }
            current.addObject(object.pose.header.frame_id, "frameid");
            current.setObjectFluent("frame-id", object.name, object.pose.header.frame_id);
            if(s_RequestGraspability) {
                current.setBooleanPredicate("graspable-from", object.name + " " + location + " left_arm",
                       object.reachable_left_arm);
                current.setBooleanPredicate("graspable-from", object.name + " " + location + " right_arm",
                       object.reachable_right_arm);
            } else {
                current.setBooleanPredicate("graspable-from", object.name + " " + location + " left_arm", true);
                current.setBooleanPredicate("graspable-from", object.name + " " + location + " right_arm", true);
            }
            current.setNumericalFluent("x", object.name, object.pose.pose.position.x);
            current.setNumericalFluent("y", object.name, object.pose.pose.position.y);
            current.setNumericalFluent("z", object.name, object.pose.pose.position.z);
            current.setNumericalFluent("qx", object.name, object.pose.pose.orientation.x);
            current.setNumericalFluent("qy", object.name, object.pose.pose.orientation.y);
            current.setNumericalFluent("qz", object.name, object.pose.pose.orientation.z);
            current.setNumericalFluent("qw", object.name, object.pose.pose.orientation.w);
            current.setNumericalFluent("timestamp", object.name, object.pose.header.stamp.toSec());
        }

        current.setBooleanPredicate("searched", location, true);
        current.setBooleanPredicate("recent-detected-objects", location, true);
    }

};

