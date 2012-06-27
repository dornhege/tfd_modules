#include "tidyup_actions/stateCreatorRobotPose.h"
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>

PLUGINLIB_DECLARE_CLASS(tidyup_actions, state_creator_robot_pose,
        tidyup_actions::StateCreatorRobotPose, continual_planning_executive::StateCreator)

namespace tidyup_actions
{

    StateCreatorRobotPose::StateCreatorRobotPose()
    {
        ros::NodeHandle nhPriv("~");
        ros::NodeHandle nh;
        nhPriv.param("nav_target_tolerance_xy", _goalToleranceXY, 0.5);
        nhPriv.param("nav_target_tolerance_yaw", _goalToleranceYaw, 0.26);  //15deg

        bool relative;
        nhPriv.param("nav_target_tolerance_relative_to_move_base", relative, false);
        if(relative) {
            // relative mode: 1. get the namespace for base_local_planner
            std::string base_local_planner_ns;
            if(!nhPriv.getParam("nav_base_local_planner_ns", base_local_planner_ns)) {
                ROS_WARN("nav_target_tolerance_relative_to_move_base set, but nav_base_local_planner_ns not set - trying to estimate");
                std::string local_planner;
                if(!nh.getParam("move_base_node/base_local_planner", local_planner)
                        && !nh.getParam("move_base/base_local_planner", local_planner)) {
                    ROS_ERROR("move_base(_node)/base_local_planner not set - falling back to absolute mode.");
                } else {
                    // dwa_local_planner/DWAPlannerROS -> DWAPlannerROS
                    std::string::size_type x = local_planner.find_last_of("/");
                    if(x == std::string::npos)
                        base_local_planner_ns = local_planner;
                    else
                        base_local_planner_ns = local_planner.substr(x + 1);
                    ROS_INFO("Estimated base_local_planner_ns to %s.", base_local_planner_ns.c_str());
                }
            }
            
            if(!base_local_planner_ns.empty()) { // success: 2. get the xy_goal_tolerance
                double move_base_tol_xy;
                if(!nh.getParam(base_local_planner_ns + "/xy_goal_tolerance", move_base_tol_xy)) {
                    ROS_ERROR_STREAM("nav_target_tolerance_relative_to_move_base was true, but "
                            << (base_local_planner_ns + "/xy_goal_tolerance") << " was not set"
                            << " - falling back to absolute mode");
                } else { // 2. add move_base's tolerance to our relative tolerance
                    _goalToleranceXY += move_base_tol_xy;
                }

                double move_base_tol_yaw;
                if(!nh.getParam(base_local_planner_ns + "/yaw_goal_tolerance", move_base_tol_yaw)) {
                    ROS_ERROR_STREAM("nav_target_tolerance_relative_to_move_base was true, but "
                            << (base_local_planner_ns + "/yaw_goal_tolerance") << " was not set"
                            << " - falling back to absolute mode");
                } else { // 2. add move_base's tolerance to our relative tolerance
                    _goalToleranceYaw += move_base_tol_yaw;
                }
            }
        }

        ROS_INFO("Tolerance for accepting nav goals set to %f m, %f deg.",
                _goalToleranceXY, angles::to_degrees(_goalToleranceYaw));

        if(s_PublishLocationsAsMarkers) {
            _markerPub = nhPriv.advertise<visualization_msgs::MarkerArray>("robot_pose_markers", 5, true);
            ROS_INFO("marker topic: %s", _markerPub.getTopic().c_str());
        }
    }

    StateCreatorRobotPose::~StateCreatorRobotPose()
    {
    }

    void StateCreatorRobotPose::initialize(const std::deque<std::string> & arguments)
    {
        ROS_ASSERT(arguments.size() == 4);

        _robotPoseObject = arguments[0];
        _robotPoseType = arguments[1];
        _atPredicate = arguments[2];
        _locationType = arguments[3];

        if(_robotPoseObject == "-")
            _robotPoseObject = "";
        if(_robotPoseType == "-")
            _robotPoseType = "";
        if(_atPredicate == "-")
            _atPredicate = "";
        if(_locationType == "-")
            _locationType = "";
    }

    bool StateCreatorRobotPose::fillState(SymbolicState & state)
    {
        tf::StampedTransform transform;
        try{
            _tf.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return false;
        }

        // 1. Real robot location
        if(!_robotPoseObject.empty()) {
            ROS_ASSERT(!_robotPoseType.empty());
            state.addObject(_robotPoseObject, _robotPoseType);
            state.setNumericalFluent("x", _robotPoseObject, transform.getOrigin().x());
            state.setNumericalFluent("y", _robotPoseObject, transform.getOrigin().y());
            state.setNumericalFluent("z", _robotPoseObject, transform.getOrigin().z());
            state.setNumericalFluent("qx", _robotPoseObject, transform.getRotation().x());
            state.setNumericalFluent("qy", _robotPoseObject, transform.getRotation().y());
            state.setNumericalFluent("qz", _robotPoseObject, transform.getRotation().z());
            state.setNumericalFluent("qw", _robotPoseObject, transform.getRotation().w());
            state.setNumericalFluent("timestamp", _robotPoseObject, ros::Time::now().toSec());
            state.addObject("/map", "frameid");
            state.setObjectFluent("frame-id", _robotPoseObject, "/map");
        }


        // 2.b check if we are at any _locations
        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> targets =
            state.getTypedObjects().equal_range(_locationType);

        vector<string> paramList;
        paramList.push_back("dummy");

        double minDist = HUGE_VAL;
        string nearestTarget = "";

        int atLocations = 0;
        for(SymbolicState::TypedObjectConstIterator it = targets.first; it != targets.second; it++) {
            string target = it->second;
            if(target == _robotPoseObject)  // skip current robot location
                continue;

            // first get xyz, qxyzw from state
            Predicate p;
            paramList[0] = target;
            p.parameters = paramList;

            p.name = "x";
            double posX;
            if(!state.hasNumericalFluent(p, &posX)) {
                ROS_ERROR("%s: target object: %s - no x-location in state.", __func__, target.c_str());
                continue;
            }
            double posY;
            p.name = "y";
            if(!state.hasNumericalFluent(p, &posY)) {
                ROS_ERROR("%s: target object: %s - no y-location in state.", __func__, target.c_str());
                continue;
            }

            double qx;
            p.name = "qx";
            if(!state.hasNumericalFluent(p, &qx)) {
                ROS_ERROR("%s: target object: %s - no qx in state.", __func__, target.c_str());
                continue;
            }
            double qy;
            p.name = "qy";
            if(!state.hasNumericalFluent(p, &qy)) {
                ROS_ERROR("%s: target object: %s - no qy in state.", __func__, target.c_str());
                continue;
            }
            double qz;
            p.name = "qz";
            if(!state.hasNumericalFluent(p, &qz)) {
                ROS_ERROR("%s: target object: %s - no qz in state.", __func__, target.c_str());
                continue;
            }
            double qw;
            p.name = "qw";
            if(!state.hasNumericalFluent(p, &qw)) {
                ROS_ERROR("%s: target object: %s - no qw in state.", __func__, target.c_str());
                continue;
            }

            // compute dXY, dYaw between current pose and target
            tf::Transform targetTransform(btQuaternion(qx, qy, qz, qw), btVector3(posX, posY, 0.0));
            tf::Transform deltaTransform = targetTransform.inverseTimes(transform);

            double dDist = hypot(deltaTransform.getOrigin().x(), deltaTransform.getOrigin().y());
            double dAng = tf::getYaw(deltaTransform.getRotation());
            ROS_INFO("Target %s dist: %f m ang: %f deg", target.c_str(), dDist, angles::to_degrees(dAng));

            if(!_atPredicate.empty()) {
                // Found a target - update state!
                if(dDist < _goalToleranceXY && fabs(dAng) < _goalToleranceYaw) {
                    ROS_INFO("(at) target %s !", target.c_str());
                    state.setBooleanPredicate(_atPredicate, target, true);
                    atLocations++;
                } else {
                    state.setBooleanPredicate(_atPredicate, target, false);
                }
                if(dDist < minDist) {
                    minDist = dDist;
                    nearestTarget = target;
                }
            }
        }

        ROS_INFO("Nearest target is %s (%f m).", nearestTarget.c_str(), minDist);

        // 2.a Set the robot pose, if we are not already at another pose
        if(!_atPredicate.empty() && !_robotPoseObject.empty()) {
            if(atLocations == 0) {
                state.setBooleanPredicate(_atPredicate, _robotPoseObject, true);
            } else {
                state.setBooleanPredicate(_atPredicate, _robotPoseObject, false);
                if(atLocations > 1) {
                    ROS_WARN("We are at %d locations at the same time!.", atLocations);
                }
            }
        }

        if(s_PublishLocationsAsMarkers)
            publishLocationsAsMarkers(state);

        return true;
    }

    /**
     * Publish markers for locations:
     * target locations are yellow or green if the robot is at the location
     * the robot location is white or blue if the robot is at the location
     */
    void StateCreatorRobotPose::publishLocationsAsMarkers(const SymbolicState & state)
    {
        if(!_markerPub) {
            ROS_WARN("%s: _markerPub invalid.", __func__);
            return;
        }

        visualization_msgs::MarkerArray ma;

        if(!_locationType.empty()) {
            // Check if we are at any grasp_locations
            pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> targets =
                state.getTypedObjects().equal_range(_locationType);

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

                p.name = _atPredicate;
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
        }

        // finally robot location marker
        if(!_robotPoseObject.empty()) {
            bool l0ok = true;
            visualization_msgs::Marker mark;
            mark.header.frame_id = "/map";
            mark.ns = "robot_location";
            mark.id = 0;
            mark.type = visualization_msgs::Marker::SPHERE;
            mark.action = visualization_msgs::Marker::ADD;

            Predicate p;
            vector<string> paramList;
            paramList[0] = _robotPoseObject;
            p.parameters = paramList;

            p.name = "x";
            double valueX;
            if(!state.hasNumericalFluent(p, &valueX)) {
                ROS_ERROR("%s: %s - no x-location.", __func__, _robotPoseObject.c_str());
                l0ok = false;
            }
            double valueY;
            p.name = "y";
            if(!state.hasNumericalFluent(p, &valueY)) {
                ROS_ERROR("%s: %s - no y-location.", __func__, _robotPoseObject.c_str());
                l0ok = false;
            }

            p.name = _atPredicate;
            bool at = false;
            if(!state.hasBooleanPredicate(p, &at)) {
                ROS_ERROR("%s: state has %s not set for %s.", __func__,
                        _atPredicate.c_str(), _robotPoseObject.c_str());
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
                mark.text = _robotPoseObject;
            } else {
                mark.action = visualization_msgs::Marker::DELETE;
            }
            ma.markers.push_back(mark);
        }

        _markerPub.publish(ma);
    }

};

