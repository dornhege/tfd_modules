#include "planner_navigation_actions/goalCreatorROSNavigation.h"
#include <pluginlib/class_list_macros.h>
#include "hardcoded_facts/geometryPoses.h"
#include <ros/ros.h>

PLUGINLIB_DECLARE_CLASS(planner_navigation_actions, goal_creator_ros_navigation,
        planner_navigation_actions::GoalCreatorROSNavigation, continual_planning_executive::GoalCreator)

namespace planner_navigation_actions
{

    GoalCreatorROSNavigation::GoalCreatorROSNavigation()
    {
    }

    GoalCreatorROSNavigation::~GoalCreatorROSNavigation()
    {
    }

    bool GoalCreatorROSNavigation::fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal)
    {
        ros::NodeHandle nhPriv("~");

        // load goal locations
        std::string goalLocationsFile;
        if(!nhPriv.getParam("goal_locations", goalLocationsFile)) {
            ROS_ERROR("Could not get ~goal_locations parameter.");
            return false;
        }
        GeometryPoses goalLocations;
        if(!goalLocations.load(goalLocationsFile)) {
            ROS_ERROR("Could not load goal locations from \"%s\".", goalLocationsFile.c_str());
            return false;
        }
        // create the actual states
        const std::map<std::string, geometry_msgs::PoseStamped> & goalPoses = goalLocations.getPoses();
        forEach(const GeometryPoses::NamedPose & np, goalPoses) {
            currentState.addObject(np.first, "target");
            goal.addObject(np.first, "target");

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
            currentState.setBooleanPredicate("explored", np.first.c_str(), false);
            goal.setBooleanPredicate("explored", np.first.c_str(), true);
        }

        return true;
    }

};

