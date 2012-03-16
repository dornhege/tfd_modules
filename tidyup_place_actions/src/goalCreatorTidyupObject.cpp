#include "tidyup_place_actions/goalCreatorTidyupObject.h"
#include "hardcoded_facts/geometryPoses.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_place_actions, goal_creator_grasp_object,
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

        // load goal locations
        std::string goalLocationsFile;
        if(!nhPriv.getParam("goal_locations", goalLocationsFile)) {
            ROS_ERROR("Could not get ~goal_locations parameter.");
            return false;
        }

        // at the targets to state and goal
        GeometryPoses goalLocations;
        if(!goalLocations.load(goalLocationsFile)) {
            ROS_ERROR("Could not load goal locations from \"%s\".", goalLocationsFile.c_str());
            return false;
        }
        // create the actual states
        const std::map<std::string, geometry_msgs::PoseStamped> & goalPoses = goalLocations.getPoses();
        forEach(const GeometryPoses::NamedPose & np, goalPoses) {
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
            // make them clean
            goal.setBooleanPredicate("clean", np.first, true);
        }

        // a bit hacky: init currentState here
        currentState.setBooleanPredicate("handFree", "right_arm", true);
        currentState.setBooleanPredicate("handFree", "right_arm", true);
        // TODO: read from params
        currentState.setBooleanPredicate("canGrasp", "left_arm", true);
        currentState.setBooleanPredicate("canGrasp", "left_arm", false);

        return true;
    }

};
