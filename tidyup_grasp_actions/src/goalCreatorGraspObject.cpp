#include "tidyup_grasp_actions/goalCreatorGraspObject.h"
#include "hardcoded_facts/geometryPoses.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_grasp_actions, goal_creator_grasp_object,
        tidyup_grasp_actions::GoalCreatorGraspObject, continual_planning_executive::GoalCreator)

namespace tidyup_grasp_actions 
{

    GoalCreatorGraspObject::GoalCreatorGraspObject()
    {
    }

    GoalCreatorGraspObject::~GoalCreatorGraspObject()
    {
    }

    bool GoalCreatorGraspObject::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
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
        const std::map<std::string, geometry_msgs::Pose> & goalPoses = goalLocations.getPoses();
        forEach(const GeometryPoses::NamedPose & np, goalPoses) {
            currentState.addObject(np.first, "grasp_location");
            goal.addObject(np.first, "grasp_location");

            currentState.setNumericalFluent("x", np.first, np.second.position.x);
            currentState.setNumericalFluent("y", np.first, np.second.position.y);
            currentState.setNumericalFluent("z", np.first, np.second.position.z);
            currentState.setNumericalFluent("qx", np.first, np.second.orientation.x);
            currentState.setNumericalFluent("qy", np.first, np.second.orientation.y);
            currentState.setNumericalFluent("qz", np.first, np.second.orientation.z);
            currentState.setNumericalFluent("qw", np.first, np.second.orientation.w);
            // make them clean
            goal.setBooleanPredicate("clean", np.first, true);
        }

        // a bit hacky: init currentState here
        // TODO: fix this, r_arm won't work for non-tuck domain any more
        currentState.addObject("right_arm", "r_arm");
        currentState.setBooleanPredicate("handFree", "right_arm", true);
        currentState.addObject("left_arm", "l_arm");
        currentState.setBooleanPredicate("handFree", "left_arm", true);

        return true;
    }

};

