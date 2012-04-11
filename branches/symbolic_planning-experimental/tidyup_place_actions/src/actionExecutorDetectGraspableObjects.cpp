#include "tidyup_place_actions/actionExecutorDetectGraspableObjects.h"
#include <pluginlib/class_list_macros.h>
#include <tidyup_msgs/RequestObjectsGraspability.h>
#include <gki_utils/stringutil.h>
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
        return true;
    }

    void ActionExecutorDetectGraspableObjects::updateState(bool success,
            tidyup_msgs::DetectGraspableObjects::Response & response,
            const DurativeAction & a, SymbolicState & current)
    {
        if(!success)
            return;

        ROS_ASSERT(a.parameters.size() == 1);
        std::string location = a.parameters.at(0);

        std::vector<tidyup_msgs::GraspableObject> objects = response.objects;

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
                it != objects.end(); it++)
        {
            tidyup_msgs::GraspableObject & object = *it;

            current.addObject(object.name, "movable_object");
            if(object.pose.header.frame_id.empty()) {
                ROS_ERROR("DetectGraspableObjects returned empty frame_id for object: %s", object.name.c_str());
                object.pose.header.frame_id = "INVALID_FRAME_ID";
            }
            current.addObject(object.pose.header.frame_id, "frameid");
            string objectPoseName = object.name + "_pose";
            current.addObject(objectPoseName, "object_pose");
            current.setObjectFluent("frame-id", objectPoseName, object.pose.header.frame_id);
            current.setNumericalFluent("x", objectPoseName, object.pose.pose.position.x);
            current.setNumericalFluent("y", objectPoseName, object.pose.pose.position.y);
            current.setNumericalFluent("z", objectPoseName, object.pose.pose.position.z);
            current.setNumericalFluent("qx", objectPoseName, object.pose.pose.orientation.x);
            current.setNumericalFluent("qy", objectPoseName, object.pose.pose.orientation.y);
            current.setNumericalFluent("qz", objectPoseName, object.pose.pose.orientation.z);
            current.setNumericalFluent("qw", objectPoseName, object.pose.pose.orientation.w);
            current.setNumericalFluent("timestamp", objectPoseName, object.pose.header.stamp.toSec());
            current.setObjectFluent("at-object", object.name, objectPoseName);
            current.setBooleanPredicate("belongs-to", objectPoseName + " "+location+"_table", true);
            // tidy-location: (tidy-location ?o ?s)
            current.setBooleanPredicate("tidy-location", object.name + " sink", true);

            // add graspable predicates from current location
            if(s_RequestGraspability)
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

            vector<string> parameters;
            parameters.push_back(object.name);
            parameters.push_back(objectPoseName);
            parameters.push_back("arm");
            parameters.push_back(location);
            // add can-putdown for current object_pose and object
            parameters[2] = "right_arm";
            current.setBooleanPredicate("can-putdown", parameters, true);
            parameters[2] = "left_arm";
            current.setBooleanPredicate("can-putdown", parameters, true);
            // for all object_locations and arms: set can-putdown for sink_location
            parameters[3] = "sink_location";
            pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> poses =
                        current.getTypedObjects().equal_range("object_pose");
            for (SymbolicState::TypedObjectConstIterator poseIt = poses.first;
                    poseIt != poses.second; poseIt++)
            {
                string pose = poseIt->second;
                if (StringUtil::startsWith(pose, "sink"))
                {
                    // (can-putdown ?o - movable_object ?p - object_pose ?a - arm ?g - grasp_location)
                    parameters[1] = poseIt->second;
                    parameters[2] = "right_arm";
                    current.setBooleanPredicate("can-putdown", parameters, true);
                    parameters[2] = "left_arm";
                    current.setBooleanPredicate("can-putdown", parameters, true);
                }
            }
            // other objects to this pose
            parameters[1] = objectPoseName;
            parameters[3] = location;
            pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> objectRange =
                    current.getTypedObjects().equal_range("movable_object");
            for (SymbolicState::TypedObjectConstIterator objectIterator = objectRange.first;
                    objectIterator != objectRange.second; objectIterator++)
            {
                // (can-putdown ?o - movable_object ?p - object_pose ?a - arm ?g - grasp_location)
                parameters[0] = objectIterator->second;
                parameters[2] = "right_arm";
                current.setBooleanPredicate("can-putdown", parameters, true);
                parameters[2] = "left_arm";
                current.setBooleanPredicate("can-putdown", parameters, true);
            }
            // current object to any other object pose
//            parameters[0] = object.name;
//            // (belongs-to ?p - object_pose ?s - static_object)
//            Predicate belongsTo;
//            belongsTo.name = "belongs-to";
//            pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> objectPoseRange =
//                current.getTypedObjects().equal_range("object_pose");
//            for (SymbolicState::TypedObjectConstIterator objectPoseIterator = objectPoseRange.first;
//                    objectPoseIterator != objectPoseRange.second; objectPoseIterator++)
//            {
//                // TODO: find corresponding grasping_location...
//                // (can-putdown ?o - movable_object ?p - object_pose ?a - arm ?g - grasp_location)
//                parameters[1] = objectPoseIterator->second;
//                parameters[2] = "right_arm";
//                current.setBooleanPredicate("can-putdown", parameters, true);
//                parameters[2] = "left_arm";
//                current.setBooleanPredicate("can-putdown", parameters, true);
//            }
        }

        // assume any movable_object can be put to any known object_pose
        // for all movable_objects object_locations and arms: add missing can-putdown predicates
//        vector<string> parameters;
//        parameters.push_back("movable_object");
//        parameters.push_back("objec_pose");
//        parameters.push_back("arm");
//        parameters.push_back("grasp_location");
//        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> objectPoseRange =
//                    current.getTypedObjects().equal_range("object_pose");
//        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> locationRange =
//                    current.getTypedObjects().equal_range("grasp_location");
//        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> objectRange =
//                    current.getTypedObjects().equal_range("movable_object");
//        for (SymbolicState::TypedObjectConstIterator objectIterator = objectRange.first;
//                objectIterator != objectRange.second; objectIterator++)
//        {
//
//
//
//            // (can-putdown ?o - movable_object ?p - object_pose ?a - arm ?g - grasp_location)
//            parameters[0] = objectIterator->second;
////            parameters[1] = poseIt->second;
//            parameters[2] = "right_arm";
//            current.setBooleanPredicate("can-putdown", parameters, true);
//            parameters[2] = "left_arm";
//            current.setBooleanPredicate("can-putdown", parameters, true);
//        }


        current.setBooleanPredicate("searched", location, true);
        current.setBooleanPredicate("recent-detected-objects", location, true);
    }

};

