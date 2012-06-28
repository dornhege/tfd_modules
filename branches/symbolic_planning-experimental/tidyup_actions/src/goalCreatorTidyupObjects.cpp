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

        // load object_locations
        std::string locationsFile;
        // load grasp_locations
        if(!nhPriv.getParam("locations", locationsFile)) {
            ROS_ERROR("Could not get ~locations parameter.");
            return false;
        }
        ROS_WARN("file_name: %s", locationsFile.c_str());
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
            currentState.addObject(location, "manipulation_location");
            goal.addObject(location, "manipulation_location");
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
            if (rooms.find(room) == rooms.end())
            {
                rooms.insert(room);
                currentState.addObject(room, "room");
            }
            currentState.setObjectFluent("location-in-room", location, room);
            if (StringUtil::startsWith(type, "table"))
            {
                if (static_objects.find(type) == static_objects.end())
                {
                    static_objects.insert(type);
                    currentState.addObject(type, "static_object");
                }
                currentState.setBooleanPredicate("static-object-at-location", type + " " + location, true);
            }
            else if (StringUtil::startsWith(type, "door"))
            {
                if (doors.find(type) == doors.end())
                {
                    doors.insert(type);
                    currentState.addObject(type, "door");
                }
                currentState.setObjectFluent("belongs-to-door", location, type);
            }
        }

        goal.setForEachGoalStatement("grasp_location", "searched", true);
        goal.setForEachGoalStatement("movable_object", "tidy", true);
        goal.setForEachGoalStatement("arm", "hand-free", true);

        currentState.setBooleanPredicate("can-grasp", "right_arm", true);
        currentState.setBooleanPredicate("can-grasp", "left_arm", true);

        currentState.setObjectFluent("arm-position", "right_arm", "arm_unknown");
        currentState.setObjectFluent("arm-position", "left_arm", "arm_unknown");

        return true;
    }

};

