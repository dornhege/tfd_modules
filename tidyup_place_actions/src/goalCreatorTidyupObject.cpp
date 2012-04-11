#include "tidyup_place_actions/goalCreatorTidyupObject.h"
#include "hardcoded_facts/geometryPoses.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_place_actions, goal_creator_tidyup_object,
        tidyup_place_actions::GoalCreatorTidyupObject, continual_planning_executive::GoalCreator)

namespace tidyup_place_actions
{

    GoalCreatorTidyupObject::GoalCreatorTidyupObject()
    {
    }

    GoalCreatorTidyupObject::~GoalCreatorTidyupObject()
    {
    }

    bool GoalCreatorTidyupObject::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
        ros::NodeHandle nhPriv("~");

        // static objects
        currentState.addObject("sink", "static_object"); // tidy location for plates & glasses

        // load object_locations
        std::string locationsFile;
        if(!nhPriv.getParam("object_locations", locationsFile)) {
            ROS_ERROR("Could not get ~object_locations parameter.");
            return false;
        }
        GeometryPoses locations;
        if(!locations.load(locationsFile)) {
            ROS_ERROR("Could not load object_locations from \"%s\".", locationsFile.c_str());
            return false;
        }
        vector<string> parameters;
        parameters.push_back("object_pose_name");
        parameters.push_back("sink");
        forEach(const GeometryPoses::NamedPose & np, locations.getPoses()) {
            currentState.addObject(np.first, "object_pose");
            goal.addObject(np.first, "object_pose");

            currentState.setNumericalFluent("timestamp", np.first, np.second.header.stamp.toSec());
            currentState.addObject(np.second.header.frame_id, "frameid");
            currentState.setObjectFluent("frame-id", np.first, np.second.header.frame_id);
            currentState.setNumericalFluent("x", np.first, np.second.pose.position.x);
            currentState.setNumericalFluent("y", np.first, np.second.pose.position.y);
            currentState.setNumericalFluent("z", np.first, np.second.pose.position.z);
            currentState.setNumericalFluent("qx", np.first, np.second.pose.orientation.x);
            currentState.setNumericalFluent("qy", np.first, np.second.pose.orientation.y);
            currentState.setNumericalFluent("qz", np.first, np.second.pose.orientation.z);
            currentState.setNumericalFluent("qw", np.first, np.second.pose.orientation.w);
            // (belongs-to ?p - object_pose ?s - static_object)
            parameters[0] = np.first;
            currentState.setBooleanPredicate("belongs-to", parameters, true);
        }
        // TODO: set tidy predicates somewhere...

        // load grasp_locations
        if(!nhPriv.getParam("grasp_locations", locationsFile)) {
            ROS_ERROR("Could not get ~grasp_locations parameter.");
            return false;
        }
        ROS_WARN("file_name: %s", locationsFile.c_str());
        locations = GeometryPoses();
        if(!locations.load(locationsFile)) {
            ROS_ERROR("Could not load grasp_locations from \"%s\".", locationsFile.c_str());
            return false;
        }
        forEach(const GeometryPoses::NamedPose & np, locations.getPoses()) {
            currentState.addObject(np.first, "grasp_location");
            goal.addObject(np.first, "grasp_location");

            currentState.setNumericalFluent("timestamp", np.first, np.second.header.stamp.toSec());
            currentState.addObject(np.second.header.frame_id, "frameid");
            currentState.setObjectFluent("frame-id", np.first, np.second.header.frame_id);
            currentState.setNumericalFluent("x", np.first, np.second.pose.position.x);
            currentState.setNumericalFluent("y", np.first, np.second.pose.position.y);
            currentState.setNumericalFluent("z", np.first, np.second.pose.position.z);
            currentState.setNumericalFluent("qx", np.first, np.second.pose.orientation.x);
            currentState.setNumericalFluent("qy", np.first, np.second.pose.orientation.y);
            currentState.setNumericalFluent("qz", np.first, np.second.pose.orientation.z);
            currentState.setNumericalFluent("qw", np.first, np.second.pose.orientation.w);
            currentState.addObject(np.first+"_table", "static_object");
        }

        goal.setForEachGoalStatement("grasp_location", "searched", true);
        goal.setForEachGoalStatement("movable_object", "tidy", true);
        goal.setForEachGoalStatement("arm", "hand-free", true);

        // a bit hacky: init currentState here
        currentState.setBooleanPredicate("hand-free", "right_arm", true);
        currentState.setBooleanPredicate("hand-free", "left_arm", true);

        currentState.setBooleanPredicate("can-grasp", "right_arm", true);
        currentState.setBooleanPredicate("can-grasp", "left_arm", true);

        currentState.setObjectFluent("arm-position", "right_arm", "unknown_armpos");
        currentState.setObjectFluent("arm-position", "left_arm", "unknown_armpos");

        return true;
    }

};

