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

        if(!_robotPoseVis.initialize()) {
            ROS_ERROR("Failed to initialized RobotPoseVisualization.");
        }
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

        double minDist = HUGE_VAL;
        string nearestTarget = "";

        int atLocations = 0;
        for(SymbolicState::TypedObjectConstIterator it = targets.first; it != targets.second; it++) {
            string target = it->second;
            if(target == _robotPoseObject)  // skip current robot location
                continue;

            geometry_msgs::PoseStamped targetPose;
            if(!extractPoseStamped(state, target, targetPose)) {
                ROS_ERROR("%s: could not extract pose for target object: %s.", __func__, target.c_str());
                continue; 
            }
            if(targetPose.header.frame_id != "/map") {
                ROS_ERROR("Target pose %s had frame-id: %s - should be /map.",
                        target.c_str(), targetPose.header.frame_id.c_str());
                continue;
            }

            // compute dXY, dYaw between current pose and target
            tf::Transform targetTransform;//(btQuaternion(qx, qy, qz, qw), btVector3(posX, posY, 0.0));
            tf::poseMsgToTF(targetPose.pose, targetTransform);
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

    bool StateCreatorRobotPose::extractPoseStamped(const SymbolicState & state, const string & object,
            geometry_msgs::PoseStamped & pose) const
    {
        bool ret = true;

        // first get xyz, qxyzw from state
        Predicate p;
        p.parameters.push_back(object);

        double posX = 0;
        p.name = "x";
        if(!state.hasNumericalFluent(p, &posX)) {
            ROS_ERROR("%s: object: %s - no x-location in state.", __func__, object.c_str());
            ret = false;;
        }
        double posY = 0;
        p.name = "y";
        if(!state.hasNumericalFluent(p, &posY)) {
            ROS_ERROR("%s: object: %s - no y-location in state.", __func__, object.c_str());
            ret = false;;
        }
        double posZ = 0;
        p.name = "z";
        if(!state.hasNumericalFluent(p, &posZ)) {
            ROS_ERROR("%s: object: %s - no z-location in state.", __func__, object.c_str());
            ret = false;;
        }

        double qx;
        p.name = "qx";
        if(!state.hasNumericalFluent(p, &qx)) {
            ROS_ERROR("%s: object: %s - no qx in state.", __func__, object.c_str());
            ret = false;;
        }
        double qy;
        p.name = "qy";
        if(!state.hasNumericalFluent(p, &qy)) {
            ROS_ERROR("%s: object: %s - no qy in state.", __func__, object.c_str());
            ret = false;;
        }
        double qz;
        p.name = "qz";
        if(!state.hasNumericalFluent(p, &qz)) {
            ROS_ERROR("%s: object: %s - no qz in state.", __func__, object.c_str());
            ret = false;;
        }
        double qw;
        p.name = "qw";
        if(!state.hasNumericalFluent(p, &qw)) {
            ROS_ERROR("%s: object: %s - no qw in state.", __func__, object.c_str());
            ret = false;;
        }

        double timestamp;
        p.name = "timestamp";
        if(!state.hasNumericalFluent(p, &timestamp)) {
            ROS_ERROR("%s: object: %s - no timestamp in state.", __func__, object.c_str());
            ret = false;;
        }

        string frameid;
        p.name = "frame-id";
        if(!state.hasObjectFluent(p, &frameid)) {
            ROS_ERROR("%s: object: %s - no frameid in state.", __func__, object.c_str());
            ret = false;;
        }

        pose.header.frame_id = frameid;
        pose.header.stamp = ros::Time(timestamp);
        pose.pose.position.x = posX;
        pose.pose.position.y = posY;
        pose.pose.position.z = posZ;
        pose.pose.orientation.x = qx;
        pose.pose.orientation.y = qy;
        pose.pose.orientation.z = qz;
        pose.pose.orientation.w = qw;

        return ret;
    }

    std_msgs::ColorRGBA StateCreatorRobotPose::getLocationColor(
            const SymbolicState & state, const string & location) const
    {
        std_msgs::ColorRGBA color;

        Predicate p;
        p.name = _atPredicate;
        p.parameters.push_back(location);
        bool at = false;
        if(!state.hasBooleanPredicate(p, &at)) {
            ROS_ERROR("%s: state has at-base not set for %s.", __func__, location.c_str());
            return color;
        }

        if(location == _robotPoseObject) {      //robot
            color.a = 0.8;
            if(at) {
                color.b = 1.0;
            } else {
                color.r = 1.0;
                color.g = 1.0;
                color.b = 1.0;
            }
        } else {            // targets
            color.a = 0.8;
            if(at) {
                color.g = 1.0;
            } else {
                color.r = 1.0;
                color.g = 1.0;
            }
        }

        return color;
    }

    // from pr2_tasks/arm_tasks.py DEFAULT_SIDE_JOINT_TRAJECTORY
    sensor_msgs::JointState StateCreatorRobotPose::getArmSideJointState() const
    {
        sensor_msgs::JointState ret;
        ret.name.resize(14);
        ret.name[0] = "l_shoulder_pan_joint";
        ret.name[1] = "l_shoulder_lift_joint";
        ret.name[2] = "l_upper_arm_roll_joint";
        ret.name[3] = "l_elbow_flex_joint";
        ret.name[4] = "l_forearm_roll_joint";
        ret.name[5] = "l_wrist_flex_joint";
        ret.name[6] = "l_wrist_roll_joint";

        ret.name[7] = "r_shoulder_pan_joint";
        ret.name[8] = "r_shoulder_lift_joint";
        ret.name[9] = "r_upper_arm_roll_joint";
        ret.name[10] = "r_elbow_flex_joint";
        ret.name[11] = "r_forearm_roll_joint";
        ret.name[12] = "r_wrist_flex_joint";
        ret.name[13] = "r_wrist_roll_joint";

        ret.position.resize(14);
        ret.position[0] = 2.1;
        ret.position[1] = 1.26;
        ret.position[2] = 1.8;
        ret.position[3] = -1.9;
        ret.position[4] = -3.5;
        ret.position[5] = -1.8;
        ret.position[6] = M_PI_2;

        ret.position[7] = -2.1;
        ret.position[8] = 1.26;
        ret.position[9] = -1.8;
        ret.position[10] = -1.9;
        ret.position[11] = 3.5;
        ret.position[12] = -1.8;
        ret.position[13] = M_PI_2;

        return ret;
    }

    visualization_msgs::MarkerArray StateCreatorRobotPose::getLocationMarkers(const SymbolicState & state,
            const string & location, const string & ns, int id, bool useMeshes) const
    {
        visualization_msgs::MarkerArray ma;

        geometry_msgs::PoseStamped locationPose;
        if(!extractPoseStamped(state, location, locationPose)) {
            ROS_ERROR("%s: could not extract pose for location object: %s.", __func__, location.c_str());
            return ma; 
        }
        if(locationPose.header.frame_id != "/map") {
            ROS_ERROR("Location pose %s had frame-id: %s - should be /map.",
                    location.c_str(), locationPose.header.frame_id.c_str());
            return ma;
        }

        if(useMeshes) {
            _robotPoseVis.updateRobotStateJoints(getArmSideJointState());
            locationPose.pose.position.z = 0;
            // zero out z to account for different frame (/base_link vs. /base_footprint)
            _robotPoseVis.updateRobotStatePose(locationPose);

            std_msgs::ColorRGBA color = getLocationColor(state, location);
            std::stringstream ss;
            ss << "mesh_" << ns << "_" << location;
            ma = _robotPoseVis.getMarkers(color, ss.str());

            return ma;
        }

        visualization_msgs::Marker mark;
        mark.header.frame_id = "/map";
        mark.ns = ns; 
        mark.id = id;
        mark.type = visualization_msgs::Marker::ARROW;
        mark.action = visualization_msgs::Marker::ADD;
        mark.pose = locationPose.pose;
        mark.pose.position.z += 0.15;
        mark.scale.x = 1.0;     // radius / 10?
        mark.scale.y = 1.0;
        mark.scale.z = 0.12;     // arrow length
        mark.color = getLocationColor(state, location);
        mark.text = location;

        ma.markers.push_back(mark);

        mark.type = visualization_msgs::Marker::SPHERE;
        mark.scale.x = 0.2;
        mark.scale.y = 0.2;
        mark.scale.z = 0.2;
        mark.id++;

        ma.markers.push_back(mark);
        
        return ma;
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

            unsigned int count = 0;
            for(SymbolicState::TypedObjectConstIterator it = targets.first; it != targets.second; it++) {
                string target = it->second;
                if(target == _robotPoseObject)  // skip current robot location
                    continue;

                visualization_msgs::MarkerArray marks = getLocationMarkers(state, target,
                        "target_locations", count, false);
                forEach(visualization_msgs::Marker & mark, marks.markers) {
                    if(mark.header.frame_id.empty())    // invalid mark
                        continue;
                    ma.markers.push_back(mark);
                }
                count += 2;
                
                if(s_PublishMeshMarkers) {
                    visualization_msgs::MarkerArray marks = getLocationMarkers(state, target,
                            "target_locations", count, true);
                    forEach(visualization_msgs::Marker & mark, marks.markers) {
                        if(mark.header.frame_id.empty())    // invalid mark
                            continue;
                        ma.markers.push_back(mark);
                    }
                }
            }

            // all should be overwritten as #targets is const, but to be safe
            for(unsigned int i = count; i < 100; i++) {
                visualization_msgs::Marker mark;
                mark.header.frame_id = "/map";
                mark.ns = "target_locations";
                mark.id = i;
                mark.action = visualization_msgs::Marker::DELETE;
                ma.markers.push_back(mark);
            }
        }

        // finally robot location marker
        if(!_robotPoseObject.empty()) {
            visualization_msgs::MarkerArray marks = getLocationMarkers(state, _robotPoseObject,
                    "robot_location", 0, false);
            forEach(visualization_msgs::Marker & mark, marks.markers) {
                if(mark.header.frame_id.empty())    // invalid mark
                    mark.action = visualization_msgs::Marker::DELETE;
                ma.markers.push_back(mark);
            }

            if(s_PublishMeshMarkers) {
                visualization_msgs::MarkerArray marks = getLocationMarkers(state, _robotPoseObject,
                        "robot_location", 0, true);
                forEach(visualization_msgs::Marker & mark, marks.markers) {
                    if(mark.header.frame_id.empty())    // invalid mark
                        continue;
                    ma.markers.push_back(mark);
                }
            }
        }

        _markerPub.publish(ma);
    }

};

