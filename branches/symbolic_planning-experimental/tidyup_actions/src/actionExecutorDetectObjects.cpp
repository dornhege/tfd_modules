#include "tidyup_actions/actionExecutorDetectObjects.h"
#include <pluginlib/class_list_macros.h>
#include <tidyup_msgs/RequestObjectsGraspability.h>

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_detect_objects,
        tidyup_actions::ActionExecutorDetectObjects,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{
    void ActionExecutorDetectObjects::initialize(const std::deque<std::string> & arguments)
    {
        ActionExecutorService<tidyup_msgs::DetectGraspableObjects>::initialize(arguments);
        requestGraspability = false;
        string graspabilityServiceName = "/learned_grasping/request_objects_graspability";
        tidyLocationName = "table1";

        if (arguments.size() >= 3)
        {
            if (arguments[2] == "NULL")
            {
                requestGraspability = false;
            } else {
                graspabilityServiceName = arguments[2];
            }
        }
        if (arguments.size() >= 4)
        {
            tidyLocationName = arguments[3];
        }

        ROS_ASSERT(_nh);
        if(requestGraspability) {
            serviceClientGraspability = _nh->serviceClient<tidyup_msgs::RequestObjectsGraspability>(
                    graspabilityServiceName);
            if(!serviceClientGraspability) {
                ROS_FATAL("Could not initialize service for RequestObjectsGraspability.");
            }
        }
    }

    bool ActionExecutorDetectObjects::fillGoal(tidyup_msgs::DetectGraspableObjects::Request & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 1);
        goal.static_object = findStaticObjectForLocation(a.parameters[0], current);
        return true;
    }

    void ActionExecutorDetectObjects::updateState(bool success,
            tidyup_msgs::DetectGraspableObjects::Response & response,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("DetectObjects returned result");
        if(success) {
            ROS_INFO("DetectObjects succeeded.");
            ROS_ASSERT(a.parameters.size() == 1);
            std::string location = a.parameters[0];
            current.setBooleanPredicate("searched", location, true);
            current.setBooleanPredicate("recent-detected-objects", location, true);

            std::vector<tidyup_msgs::GraspableObject>& objects = response.objects;
            if(requestGraspability) {
                ROS_INFO("Requesting graspability.");
                tidyup_msgs::RequestObjectsGraspability request;
                request.request.objects = objects;
                if(!serviceClientGraspability.call(request)) {
                    ROS_ERROR("Failed to call RequestObjectsGraspability service.");
                } else {
                    objects = request.response.objects;
                }
            }

            // find correct static object and set the "on" predicate
            string static_object = findStaticObjectForLocation(location, current);
            ROS_ASSERT(static_object != "");

            // remove objects form state, which were previously detected from this location
            Predicate p;
            p.name = "detected-from";
            p.parameters.pop_back();
            pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> objectRange =
                    current.getTypedObjects().equal_range("movable_object");
            for (SymbolicState::TypedObjectConstIterator objectIterator = objectRange.first;
                    objectIterator != objectRange.second; objectIterator++)
            {
                string object = objectIterator->second;
                p.parameters[0] = object;
                string detection_location;
                if (current.hasObjectFluent(p, &detection_location))
                {
                    if (location == detection_location)
                    {
                        current.removeObject(object, true);
                    }
                }
            }

            for(std::vector<tidyup_msgs::GraspableObject>::iterator it = objects.begin(); it != objects.end(); it++)
            {
                tidyup_msgs::GraspableObject & object = *it;
                current.addObject(object.name, "movable_object");
                if(object.pose.header.frame_id.empty()) {
                    ROS_ERROR("DetectGraspableObjects returned empty frame_id for object: %s", object.name.c_str());
                    object.pose.header.frame_id = "INVALID_FRAME_ID";
                }
                current.addObject(object.pose.header.frame_id, "frameid");
                current.addObject(object.name, "pose");
                current.setObjectFluent("frame-id", object.name, object.pose.header.frame_id);
                current.setNumericalFluent("x", object.name, object.pose.pose.position.x);
                current.setNumericalFluent("y", object.name, object.pose.pose.position.y);
                current.setNumericalFluent("z", object.name, object.pose.pose.position.z);
                current.setNumericalFluent("qx", object.name, object.pose.pose.orientation.x);
                current.setNumericalFluent("qy", object.name, object.pose.pose.orientation.y);
                current.setNumericalFluent("qz", object.name, object.pose.pose.orientation.z);
                current.setNumericalFluent("qw", object.name, object.pose.pose.orientation.w);
                current.setNumericalFluent("timestamp", object.name, object.pose.header.stamp.toSec());
                current.setBooleanPredicate("on", object.name + " " + static_object, true);
                current.setObjectFluent("object-detected-from", object.name, location);
                // tidy-location: (tidy-location ?o ?s)
                current.setBooleanPredicate("tidy-location", object.name + " " + tidyLocationName, true);

                // add graspable predicates from current location
                if(requestGraspability || true)
                {
                    current.setBooleanPredicate("graspable-from", object.name + " " + location + " left_arm",
                           object.reachable_left_arm);
                    current.setBooleanPredicate("graspable-from", object.name + " " + location + " right_arm",
                           object.reachable_right_arm);
                }
                else
                {
                    current.setBooleanPredicate("graspable-from", object.name + " " + location + " left_arm", true);
                    current.setBooleanPredicate("graspable-from", object.name + " " + location + " right_arm", true);
                }
            }
        }
    }

    std::string ActionExecutorDetectObjects::findStaticObjectForLocation(const std::string& location, const SymbolicState & current) const
    {
        Predicate p;
        string static_object;
        p.name = "static-object-at-location";
        p.parameters.push_back("object");
        p.parameters.push_back(location);
        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> objectRange =
                current.getTypedObjects().equal_range("static_object");
        for (SymbolicState::TypedObjectConstIterator objectIterator = objectRange.first;
                objectIterator != objectRange.second; objectIterator++)
        {
            string object = objectIterator->second;
            p.parameters[0] = object;
            bool value = false;
            if (current.hasBooleanPredicate(p, &value))
            {
                if (value)
                {
                    static_object = object;
                    break;
                }
            }
        }
        return static_object;
    }

};

