#include "tidyup_actions/actionExecutorDetectObjects.h"
#include <pluginlib/class_list_macros.h>
#include <tidyup_msgs/RequestObjectsGraspability.h>
#include "tidyup_utils/planning_scene_interface.h"

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
        if(!PlanningSceneInterface::instance()->resetPlanningScene())   // FIXME try anyways?
            ROS_ERROR("%s: PlanningScene reset failed.", __PRETTY_FUNCTION__);

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
            if(requestGraspability && false) {
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
            p.name = "on";
            p.parameters.push_back("object");
            p.parameters.push_back(static_object);
            pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> objectRange =
                    current.getTypedObjects().equal_range("movable_object");
            for (SymbolicState::TypedObjectConstIterator objectIterator = objectRange.first;
                    objectIterator != objectRange.second; objectIterator++)
            {
                string object = objectIterator->second;
                p.parameters[0] = object;
                bool onStatic = false;
                if(current.hasBooleanPredicate(p, &onStatic))
                {
                    if(onStatic) {
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
                //current.setObjectFluent("object-detected-from", object.name, location);
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

            std::set<std::string> wipeGoalNames;
            for(std::vector<tidyup_msgs::WipeGoal>::iterator it = response.wipe_goals.begin(); it != response.wipe_goals.end(); it++) {
                tidyup_msgs::WipeGoal & wg = *it;
                wipeGoalNames.insert(wg.name);

                // add/update this wipe goal
                current.addObject(wg.name, "wipe_point");   // this should never hurt, even if it's in there
                if(wg.spot.header.frame_id.empty()) {
                    ROS_ERROR("DetectGraspableObjects returned empty frame_id for wipe goal: %s", wg.name.c_str());
                    wg.spot.header.frame_id = "INVALID_FRAME_ID";
                }
                current.addObject(wg.spot.header.frame_id, "frameid");
                current.setObjectFluent("frame-id", wg.name, wg.spot.header.frame_id);
                current.setNumericalFluent("x", wg.name, wg.spot.point.x);
                current.setNumericalFluent("y", wg.name, wg.spot.point.y);
                current.setNumericalFluent("z", wg.name, wg.spot.point.z);
                current.setNumericalFluent("timestamp", wg.name, wg.spot.header.stamp.toSec());
                // if not known, set initially false
                Predicate wipedPred;
                wipedPred.name = "wiped";
                wipedPred.parameters.push_back(wg.name);
                bool wiped;
                if(!current.hasBooleanPredicate(wipedPred, &wiped)) {   // only set if unknown
                    current.setBooleanPredicate("wiped", wg.name, false);
                }
                current.setBooleanPredicate("wipe-point-on", wg.name + " " + static_object, true);
            }

            // remove all the wipe goals at this static object that were not send in this detection
            // (i.e. that are not in wipeGoalNames)
            pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> wipePointRange =
                current.getTypedObjects().equal_range("wipe_point");
            std::set<std::string> nonMatchedWipePoints;
            for(SymbolicState::TypedObjectConstIterator objectIterator = wipePointRange.first;
                    objectIterator != wipePointRange.second; objectIterator++) {
                // first check if this object is a wipe-point-on static_object
                Predicate p;
                p.name = "wipe-point-on";
                string object = objectIterator->second;
                p.parameters.push_back(object);
                p.parameters.push_back(static_object);

                bool wpOn = false;
                if(!current.hasBooleanPredicate(p, &wpOn))
                    continue;
                if(!wpOn)
                    continue;

                // it is on static_object, next check if it was contained in this detection
                if(wipeGoalNames.find(object) == wipeGoalNames.end()) {
                    nonMatchedWipePoints.insert(object);
                }
            }
            for(std::set<std::string>::iterator it = nonMatchedWipePoints.begin(); it != nonMatchedWipePoints.end(); it++) {
                ROS_WARN("Removing non matched wipe point: %s.", it->c_str());
                current.removeObject(*it, false);    // FIXME: do not remove the predicates
                // this will remember that we wiped a spot, when we for some reason manage
                // to find it again
            }

            if(!PlanningSceneInterface::instance()->resetPlanningScene())   // FIXME try anyways?
              ROS_ERROR("%s: PlanningScene reset failed.", __PRETTY_FUNCTION__);
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

