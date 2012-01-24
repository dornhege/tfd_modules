#include "tidyup_grasp_actions/stateCreatorGraspObject.h"
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>

PLUGINLIB_DECLARE_CLASS(tidyup_grasp_actions, state_creator_grasp_object,
        tidyup_grasp_actions::StateCreatorGraspObject, continual_planning_executive::StateCreator)

namespace tidyup_grasp_actions 
{

    StateCreatorGraspObject::StateCreatorGraspObject()
    {
        ros::NodeHandle nhPriv("~");
        nhPriv.param("nav_target_tolerance", _goalTolerance, 0.5);

        bool relative;
        nhPriv.param("nav_target_tolerance_relative_to_move_base", relative, false);
        if(relative) {
            // relative mode: 1. get the namespace for base_local_planner
            std::string base_local_planner_ns;
            if(!nhPriv.getParam("nav_base_local_planner_ns", base_local_planner_ns)) {
                ROS_ERROR("nav_target_tolerance_relative_to_move_base was true, but nav_base_local_planner_ns is not set - falling back to absolute mode.");
            } else { // success: 2. get the xy_goal_tolerance
                double move_base_tol;
                ros::NodeHandle nh;
                if(!nh.getParam(base_local_planner_ns + "/xy_goal_tolerance", move_base_tol)) {
                    ROS_ERROR_STREAM("nav_target_tolerance_relative_to_move_base was true, but "
                            << (base_local_planner_ns + "/xy_goal_tolerance") << " was not set"
                            << " - falling back to absolute mode");
                } else { // 2. add move_base's tolerance to our relative tolerance
                    _goalTolerance += move_base_tol;
                }
            }
        }

        ROS_INFO("Tolerance for accepting nav goals set to %f.", _goalTolerance);

        if(s_PublishLocationsAsMarkers) {
            _markerPub = nhPriv.advertise<visualization_msgs::MarkerArray>("location_markers", 5);
        }
    }

    StateCreatorGraspObject::~StateCreatorGraspObject()
    {
    }

    bool StateCreatorGraspObject::fillState(SymbolicState & state)
    {
        tf::StampedTransform transform;
        try{
            _tf.lookupTransform("/map", "/base_link",  
                    ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return false;
        }

        // Check if we are at any grasp_locations 
        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> targets = 
            state.getTypedObjects().equal_range("grasp_location");

        vector<string> paramList;
        paramList.push_back("dummy");

        double minDist = HUGE_VAL;
        string nearestTarget = "";

        int atGraspLocations = 0;
        for(SymbolicState::TypedObjectConstIterator it = targets.first; it != targets.second; it++) {
            string target = it->second;
            Predicate p;
            paramList[0] = target;
            p.parameters = paramList;

            p.name = "x";
            double valueX;
            if(!state.hasNumericalFluent(p, &valueX)) {
                ROS_ERROR("%s: target object: %s - no x-location in goal.", __func__, target.c_str());
                continue;
            }
            double valueY;
            p.name = "y";
            if(!state.hasNumericalFluent(p, &valueY)) {
                ROS_ERROR("%s: target object: %s - no y-location in goal.", __func__, target.c_str());
                continue;
            }

            double dx = transform.getOrigin().x() - valueX;
            double dy = transform.getOrigin().y() - valueY;
            // Found a target - update state!
            if(hypot(dx, dy) < _goalTolerance) {
                ROS_INFO("(at) target %s !", target.c_str());
                state.setBooleanPredicate("at-base", target, true);
                atGraspLocations++;
            } else {
                state.setBooleanPredicate("at-base", target, false);
            }
            if(hypot(dx, dy) < minDist) {
                minDist = hypot(dx, dy);
                nearestTarget = target;
            }
        }
        ROS_INFO("Nearest target is %s (%f m).", nearestTarget.c_str(), minDist);

        // Real robot location
        state.addObject("l0", "location");
        state.setNumericalFluent("x", "l0", transform.getOrigin().x());
        state.setNumericalFluent("y", "l0", transform.getOrigin().y());
        state.setNumericalFluent("z", "l0", transform.getOrigin().z());
        state.setNumericalFluent("qx", "l0", transform.getRotation().x());
        state.setNumericalFluent("qy", "l0", transform.getRotation().y());
        state.setNumericalFluent("qz", "l0", transform.getRotation().z());
        state.setNumericalFluent("qw", "l0", transform.getRotation().w());
        state.setNumericalFluent("timestamp", "l0", ros::Time::now().toSec());
        state.addObject("/map", "frameid");
        state.setObjectFluent("frame-id", "l0", "/map");

        if(atGraspLocations == 0) {
            state.setBooleanPredicate("at-base", "l0", true);
        } else {
            state.setBooleanPredicate("at-base", "l0", false);    //hm what do we give in the cost module here?
            if(atGraspLocations > 1) {
                ROS_WARN("We are at %d grasp locations at the same time!.", atGraspLocations);
            }
        }

        // can-navigate is always true for now
        for(SymbolicState::TypedObjectConstIterator it = targets.first; it != targets.second; it++) {
            string target = it->second;
            for(SymbolicState::TypedObjectConstIterator it2 = targets.first; it2 != targets.second; it2++) {
                string target2 = it2->second;
                state.setBooleanPredicate("can-navigate", target + " " + target2, true);
            }
            if(atGraspLocations == 0)
                state.setBooleanPredicate("can-navigate", target + " l0", true);
        }

        if(s_PublishLocationsAsMarkers)
            publishLocationsAsMarkers(state);

        return true;
    }

    /**
     * Publish markers for locations:
     * grasp_locations are yellow or green if the robot is at the location
     * the robot location is white or blue if the robot is at the location
     */
    void StateCreatorGraspObject::publishLocationsAsMarkers(const SymbolicState & state)
    {
        if(!_markerPub) {
            ROS_WARN("%s: _markerPub invalid.", __func__);
            return;
        }

        visualization_msgs::MarkerArray ma;

        // Check if we are at any grasp_locations 
        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> targets = 
            state.getTypedObjects().equal_range("grasp_location");

        vector<string> paramList;
        paramList.push_back("dummy");

        unsigned int count = 0;
        for(SymbolicState::TypedObjectConstIterator it = targets.first; it != targets.second; it++) {
            string target = it->second;
            Predicate p;
            paramList[0] = target;
            p.parameters = paramList;

            p.name = "x";
            double valueX;
            if(!state.hasNumericalFluent(p, &valueX)) {
                ROS_ERROR("%s: target object: %s - no x-location.", __func__, target.c_str());
                continue;
            }
            double valueY;
            p.name = "y";
            if(!state.hasNumericalFluent(p, &valueY)) {
                ROS_ERROR("%s: target object: %s - no y-location.", __func__, target.c_str());
                continue;
            }
            
            p.name = "at-base";
            bool at = false;
            if(!state.hasBooleanPredicate(p, &at)) {
                ROS_ERROR("%s: state has at-base not set for %s.", __func__, target.c_str());
                continue;
            }

            // we now know for this target it's x/y coords and if the robot is at this target
            visualization_msgs::Marker mark;
            mark.header.frame_id = "/map";
            mark.ns = "target_locations";
            mark.id = count;
            count++;
            mark.type = visualization_msgs::Marker::SPHERE;
            mark.action = visualization_msgs::Marker::ADD;
            mark.pose.position.x = valueX;
            mark.pose.position.y = valueY;
            mark.pose.position.z = 0.15;
            mark.pose.orientation.w = 1.0;
            mark.scale.x = 0.3;
            mark.scale.y = 0.3;
            mark.scale.z = 0.3;
            mark.color.a = 1.0;
            if(at) {
                mark.color.g = 1.0;
            } else {
                mark.color.r = 1.0;
                mark.color.g = 1.0;
            }
            mark.text = target;
            ma.markers.push_back(mark);
        }
        // all should be overwritten as #targets is const, but to be safe
        for(unsigned int i = count; i < 100; i++) {
            visualization_msgs::Marker mark;
            mark.header.frame_id = "/map";
            mark.ns = "target_locations";
            mark.id = i;
            mark.type = visualization_msgs::Marker::SPHERE;
            mark.action = visualization_msgs::Marker::DELETE;
            ma.markers.push_back(mark);
        }

        // finally robot location marker

        bool l0ok = true;
        visualization_msgs::Marker mark;
        mark.header.frame_id = "/map";
        mark.ns = "robot_location";
        mark.id = 0;
        mark.type = visualization_msgs::Marker::SPHERE;
        mark.action = visualization_msgs::Marker::ADD;

        Predicate p;
        paramList[0] = "l0";
        p.parameters = paramList;

        p.name = "x";
        double valueX;
        if(!state.hasNumericalFluent(p, &valueX)) {
            ROS_ERROR("%s: l0 - no x-location.", __func__);
            l0ok = false;
        }
        double valueY;
        p.name = "y";
        if(!state.hasNumericalFluent(p, &valueY)) {
            ROS_ERROR("%s: l0 - no y-location.", __func__);
            l0ok = false;
        }

        p.name = "at-base";
        bool at = false;
        if(!state.hasBooleanPredicate(p, &at)) {
            ROS_ERROR("%s: state has at-base not set for l0.", __func__);
            l0ok = false;
        }

        if(l0ok) {
            mark.pose.position.x = valueX;
            mark.pose.position.y = valueY;
            mark.pose.position.z = 0.15;
            mark.pose.orientation.w = 1.0;
            mark.scale.x = 0.3;
            mark.scale.y = 0.3;
            mark.scale.z = 0.3;
            mark.color.a = 1.0;
            if(at) {
                mark.color.b = 1.0;
            } else {
                mark.color.r = 1.0;
                mark.color.g = 1.0;
                mark.color.b = 1.0;
            }
            mark.text = "l0";
        } else {
            mark.action = visualization_msgs::Marker::DELETE;
        }
        ma.markers.push_back(mark);

        _markerPub.publish(ma);
    }

};
