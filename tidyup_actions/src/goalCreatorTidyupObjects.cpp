#include "tidyup_actions/goalCreatorTidyupObjects.h"
#include "hardcoded_facts/geometryPoses.h"
#include "gki_utils/stringutil.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <set>

PLUGINLIB_DECLARE_CLASS(tidyup_actions, goal_creator_tidyup_objects,
        tidyup_actions::GoalCreatorTidyupObjects, continual_planning_executive::GoalCreator)

namespace tidyup_actions
{

    GoalCreatorTidyupObjects::GoalCreatorTidyupObjects()
    {
    }

    GoalCreatorTidyupObjects::~GoalCreatorTidyupObjects()
    {
    }

    bool GoalCreatorTidyupObjects::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
        ros::NodeHandle nhPriv("~");

        // first add the type hierarchy
        currentState.addSuperType("pose", "pose");
        currentState.addSuperType("frameid", "frameid");
        currentState.addSuperType("location", "pose");
        currentState.addSuperType("manipulation_location", "location");
        currentState.addSuperType("door_location", "location");
        currentState.addSuperType("room", "room");
        currentState.addSuperType("static_object", "static_object");
        currentState.addSuperType("door", "door");
        currentState.addSuperType("movable_object", "pose");
        currentState.addSuperType("arm", "arm");
        currentState.addSuperType("arm_state", "arm_state");
        goal.addSuperType("pose", "pose");
        goal.addSuperType("frameid", "frameid");
        goal.addSuperType("location", "pose");
        goal.addSuperType("manipulation_location", "location");
        goal.addSuperType("door_location", "location");
        goal.addSuperType("room", "room");
        goal.addSuperType("static_object", "static_object");
        goal.addSuperType("door", "door");
        goal.addSuperType("movable_object", "pose");
        goal.addSuperType("arm", "arm");
        goal.addSuperType("arm_state", "arm_state");

        currentState.printSuperTypes();

        // load object_locations
        std::string locationsFile;
        // load grasp_locations
        if(!nhPriv.getParam("locations", locationsFile)) {
            ROS_ERROR("Could not get ~locations parameter.");
            return false;
        }
        ROS_INFO("%s: file_name: %s", __PRETTY_FUNCTION__, locationsFile.c_str());
        GeometryPoses locations = GeometryPoses();
        if(!locations.load(locationsFile)) {
            ROS_ERROR("Could not load locations from \"%s\".", locationsFile.c_str());
            return false;
        }

        std::set<string> rooms;
        std::set<string> doors;
        std::set<string> static_objects;
        forEach(const GeometryPoses::NamedPose & np, locations.getPoses()) {
            const string& location = np.first;
            currentState.setNumericalFluent("timestamp", location, np.second.header.stamp.toSec());
            currentState.addObject(np.second.header.frame_id, "frameid");
            currentState.setObjectFluent("frame-id", location, np.second.header.frame_id);
            currentState.setNumericalFluent("x", location, np.second.pose.position.x);
            currentState.setNumericalFluent("y", location, np.second.pose.position.y);
            currentState.setNumericalFluent("z", location, np.second.pose.position.z);
            currentState.setNumericalFluent("qx", location, np.second.pose.orientation.x);
            currentState.setNumericalFluent("qy", location, np.second.pose.orientation.y);
            currentState.setNumericalFluent("qz", location, np.second.pose.orientation.z);
            currentState.setNumericalFluent("qw", location, np.second.pose.orientation.w);

            // additional fluents
            // location name scheme: <type>typeName_AdditionalName_<room>roomName
            const vector<string>& nameParts = StringUtil::split(location, "_");
            const string& room = nameParts[nameParts.size()-1];
            const string& type = nameParts[0];
            rooms.insert(room);
            currentState.addObject(room, "room");
            currentState.setObjectFluent("location-in-room", location, room);

            if (StringUtil::startsWith(type, "door"))
            {
                doors.insert(type);
                currentState.addObject(type, "door");
                currentState.setObjectFluent("belongs-to-door", location, type);
                currentState.addObject(location, "door_location");
                goal.addObject(location, "door_location");
            }
            else
            {
                static_objects.insert(type);
                currentState.addObject(type, "static_object");
                currentState.setBooleanPredicate("static-object-at-location", type + " " + location, true);
                currentState.addObject(location, "manipulation_location");
                goal.addObject(location, "manipulation_location");
            }
        }

        goal.setForEachGoalStatement("manipulation_location", "searched", true);
        goal.setForEachGoalStatement("movable_object", "tidy", true);
        goal.setForEachGoalStatement("arm", "hand-free", true);

        currentState.setBooleanPredicate("can-grasp", "right_arm", true);
        currentState.setBooleanPredicate("can-grasp", "left_arm", true);

        currentState.setObjectFluent("arm-state", "right_arm", "arm_unknown");
        currentState.setObjectFluent("arm-state", "left_arm", "arm_unknown");

        return true;
    }

};

