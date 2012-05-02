#include "tidyup_fleximove_actions/goalCreatorTidyupObjects.h"
#include "hardcoded_facts/geometryPoses.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#ifdef __CDT_PARSER__
#define forEach(a, b) for(a : b)
#else
#define forEach BOOST_FOREACH
#endif
PLUGINLIB_DECLARE_CLASS(tidyup_fleximove_actions, goal_creator_tidyup_objects,
        tidyup_fleximove_actions::GoalCreatorTidyupObjects, continual_planning_executive::GoalCreator)

namespace tidyup_fleximove_actions
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

        // static objects
        currentState.addObject("sink", "static_object"); // tidy location for plates & glasses

        // load object_locations
        std::string locationsFile;
        if(!nhPriv.getParam("object_locations", locationsFile))
        {
            ROS_ERROR("Could not get ~object_locations parameter.");
            return false;
        }
        GeometryPoses locations;
        if(!locations.load(locationsFile))
        {
            ROS_ERROR("Could not load object_locations from \"%s\".", locationsFile.c_str());
            return false;
        }
        forEach(const GeometryPoses::NamedPose & np, locations.getPoses())
        {
            string object_pose = np.first+"_pose";
            currentState.addObject(np.first, "static_object");
            currentState.addObject(object_pose, "object_pose");
            currentState.setObjectFluent("object-pose", np.first, object_pose);
            goal.addObject(np.first, "static_object");

            currentState.setNumericalFluent("timestamp", object_pose, np.second.header.stamp.toSec());
            currentState.addObject(np.second.header.frame_id, "frameid");
            currentState.setObjectFluent("frame-id", object_pose, np.second.header.frame_id);
            currentState.setNumericalFluent("x", object_pose, np.second.pose.position.x);
            currentState.setNumericalFluent("y", object_pose, np.second.pose.position.y);
            currentState.setNumericalFluent("z", object_pose, np.second.pose.position.z);
            currentState.setNumericalFluent("qx", object_pose, np.second.pose.orientation.x);
            currentState.setNumericalFluent("qy", object_pose, np.second.pose.orientation.y);
            currentState.setNumericalFluent("qz", object_pose, np.second.pose.orientation.z);
            currentState.setNumericalFluent("qw", object_pose, np.second.pose.orientation.w);
        }

        goal.setForEachGoalStatement("static_object", "searched", true);
        goal.setForEachGoalStatement("movable_object", "tidy", true);
        goal.setForEachGoalStatement("arm", "hand-free", true);

        // a bit hacky: init currentState here
        currentState.setBooleanPredicate("can-grasp", "right_arm", true);
        currentState.setBooleanPredicate("can-grasp", "left_arm", true);

        currentState.setObjectFluent("arm-state", "right_arm", "arm_unknown");
        currentState.setObjectFluent("arm-state", "left_arm", "arm_unknown");

        return true;
    }

};

