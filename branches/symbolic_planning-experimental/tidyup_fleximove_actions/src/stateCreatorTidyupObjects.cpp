#include "tidyup_fleximove_actions/stateCreatorTidyupObjects.h"
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>

PLUGINLIB_DECLARE_CLASS(tidyup_place_actions, state_creator_tidyup_object,
        tidyup_place_actions::StateCreatorTidyupObjects,
        continual_planning_executive::StateCreator)

namespace tidyup_place_actions
{

StateCreatorTidyupObjects::StateCreatorTidyupObjects()
{
    ros::NodeHandle nhPriv("~");
    nhPriv.param("nav_target_tolerance_xy", _goalToleranceXY, 0.5);
    nhPriv.param("nav_target_tolerance_yaw", _goalToleranceYaw, 0.26); //15deg

    bool relative;
    nhPriv.param("nav_target_tolerance_relative_to_move_base", relative, false);
    if (relative)
    {
        // relative mode: 1. get the namespace for base_local_planner
        std::string base_local_planner_ns;
        if (!nhPriv.getParam("nav_base_local_planner_ns",
                base_local_planner_ns))
        {
            ROS_ERROR("nav_target_tolerance_relative_to_move_base was true, but nav_base_local_planner_ns is not set - falling back to absolute mode.");
        }
        else
        { // success: 2. get the xy_goal_tolerance
            double move_base_tol_xy;
            ros::NodeHandle nh;
            if (!nh.getParam(base_local_planner_ns + "/xy_goal_tolerance",
                    move_base_tol_xy))
            {
                ROS_ERROR_STREAM("nav_target_tolerance_relative_to_move_base was true, but " << (base_local_planner_ns + "/xy_goal_tolerance") << " was not set" << " - falling back to absolute mode");
            }
            else
            { // 2. add move_base's tolerance to our relative tolerance
                _goalToleranceXY += move_base_tol_xy;
            }

            double move_base_tol_yaw;
            if (!nh.getParam(base_local_planner_ns + "/yaw_goal_tolerance",
                    move_base_tol_yaw))
            {
                ROS_ERROR_STREAM("nav_target_tolerance_relative_to_move_base was true, but " << (base_local_planner_ns + "/yaw_goal_tolerance") << " was not set" << " - falling back to absolute mode");
            }
            else
            { // 2. add move_base's tolerance to our relative tolerance
                _goalToleranceYaw += move_base_tol_yaw;
            }
        }
    }

    ROS_INFO("Tolerance for accepting nav goals set to %f m, %f deg.", _goalToleranceXY, angles::to_degrees(_goalToleranceYaw));

    if (s_PublishLocationsAsMarkers)
    {
        _markerPub = nhPriv.advertise<visualization_msgs::MarkerArray>(
                "location_markers", 5, true);
        ROS_INFO("marker topic: %s", _markerPub.getTopic().c_str());
    }
}

StateCreatorTidyupObjects::~StateCreatorTidyupObjects()
{
}

bool StateCreatorTidyupObjects::fillState(SymbolicState & state)
{
    tf::StampedTransform transform;
    try
    {
        _tf.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    } catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    // Real robot location
    string robot_pose = "robot_pose";
    state.addObject("robot_pose", "pose");
    state.setNumericalFluent("x", robot_pose, transform.getOrigin().x());
    state.setNumericalFluent("y", robot_pose, transform.getOrigin().y());
    state.setNumericalFluent("z", robot_pose, transform.getOrigin().z());
    state.setNumericalFluent("qx", robot_pose, transform.getRotation().x());
    state.setNumericalFluent("qy", robot_pose, transform.getRotation().y());
    state.setNumericalFluent("qz", robot_pose, transform.getRotation().z());
    state.setNumericalFluent("qw", robot_pose, transform.getRotation().w());
    state.setNumericalFluent("timestamp", robot_pose, ros::Time::now().toSec());
    state.addObject("/map", "frameid");
    state.setObjectFluent("frame-id", robot_pose, "/map");

    if (s_PublishLocationsAsMarkers)
        publishLocationsAsMarkers(state);

    return true;
}

/**
 * Publish markers for locations:
 * grasp_locations are yellow or green if the robot is at the location
 * the robot location is white or blue if the robot is at the location
 * TODO: add tidy_locations
 */
void StateCreatorTidyupObjects::publishLocationsAsMarkers(
        const SymbolicState & state)
{
    if (!_markerPub)
    {
        ROS_WARN("%s: _markerPub invalid.", __func__);
        return;
    }

    visualization_msgs::MarkerArray ma;

    // Check if we are at any grasp_locations
    pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> static_objects =
            state.getTypedObjects().equal_range("grasp_location");

    vector<string> paramList;
    paramList.push_back("dummy");

    unsigned int count = 0;
    for (SymbolicState::TypedObjectConstIterator it = static_objects.first;
            it != static_objects.second; it++)
    {
        string static_object = it->second;
        string object_pose = static_object + "_pose";
        Predicate p;
        paramList[0] = object_pose;
        p.parameters = paramList;

        p.name = "x";
        double valueX;
        if (!state.hasNumericalFluent(p, &valueX))
        {
            ROS_ERROR("%s: target object: %s - no x-location.", __func__, object_pose.c_str());
            continue;
        }
        double valueY;
        p.name = "y";
        if (!state.hasNumericalFluent(p, &valueY))
        {
            ROS_ERROR("%s: target object: %s - no y-location.", __func__, object_pose.c_str());
            continue;
        }

        // check if target is currently in-reach / in-view
        bool searched = false;
        p.name = "searched";
        p.parameters[0] = static_object;
        state.hasBooleanPredicate(p, &searched);
        bool inView = false;

        // we now know for this target it's x/y coords and if the robot is at this target
        visualization_msgs::Marker mark;
        mark.header.frame_id = "/map";
        mark.ns = "object_locations";
        mark.id = count;
        count++;
        mark.type = visualization_msgs::Marker::ARROW;
        mark.action = visualization_msgs::Marker::ADD;
        mark.pose.position.x = valueX;
        mark.pose.position.y = valueY;
        mark.pose.position.z = 0.15;
        mark.pose.orientation.w = 1.0;
        mark.scale.x = 0.3;
        mark.scale.y = 0.3;
        mark.scale.z = 0.3;
        mark.color.a = 1.0;
        if (inView)
        {
            // green
            mark.color.g = 1.0;
        }
        else if (searched)
        {
            // gray
            mark.color.r = 0.4;
            mark.color.g = 0.4;
            mark.color.b = 0.4;
        }
        else
        {
            // yellow
            mark.color.r = 1.0;
            mark.color.g = 1.0;
        }
        mark.text = static_object;
        ma.markers.push_back(mark);
    }
    // all should be overwritten as #targets is const, but to be safe
    for (unsigned int i = count; i < 100; i++)
    {
        visualization_msgs::Marker mark;
        mark.header.frame_id = "/map";
        mark.ns = "object_locations";
        mark.id = i;
        mark.type = visualization_msgs::Marker::ARROW;
        mark.action = visualization_msgs::Marker::DELETE;
        ma.markers.push_back(mark);
    }

    // finally robot location marker

    visualization_msgs::Marker mark;
    mark.header.frame_id = "/map";
    mark.ns = "robot_location";
    mark.id = 0;
    mark.type = visualization_msgs::Marker::ARROW;
    mark.action = visualization_msgs::Marker::ADD;

    Predicate p;
    paramList[0] = "robot_pose";
    p.parameters = paramList;

    p.name = "x";
    double valueX;
    if (!state.hasNumericalFluent(p, &valueX))
    {
        ROS_ERROR("%s: l0 - no x-location.", __func__);
    }
    double valueY;
    p.name = "y";
    if (!state.hasNumericalFluent(p, &valueY))
    {
        ROS_ERROR("%s: l0 - no y-location.", __func__);
    }

    mark.pose.position.x = valueX;
    mark.pose.position.y = valueY;
    mark.pose.position.z = 0.15;
    mark.pose.orientation.w = 1.0;
    mark.scale.x = 0.3;
    mark.scale.y = 0.3;
    mark.scale.z = 0.3;
    mark.color.a = 1.0;
    mark.color.r = 1.0;
    mark.color.g = 1.0;
    mark.color.b = 1.0;
    mark.text = "robot_pose";
    ma.markers.push_back(mark);

    _markerPub.publish(ma);
}

}
;

