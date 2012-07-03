#ifndef ROBOT_POSE_VISUALIZATION_H
#define ROBOT_POSE_VISUALIZATION_H

#include <ros/ros.h>
#include <arm_navigation_msgs/RobotState.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

/// Fancy visualization for robot poses
/**
 * This is based on Sushi's state_transformer/GetRobotMarker service to 
 * acquire robot markers for a RobotState.
 *
 * To initialize the RobotState the environment server is queried once and whatever
 * is in there will be used to initialize the robot state.
 *
 * From then on the robot state can be updated with new joint states for the arms
 * or the actual robot pose.
 */
class RobotPoseVisualization
{
   public:
      RobotPoseVisualization();
      ~RobotPoseVisualization();

      /// Get the initial robot state and setup services.
      bool initialize();

      /// Reset the current robot state to the initial one.
      void resetRobotState();

      /// Update the current robot state with the joint positions from js.
      void updateRobotStateJoints(const sensor_msgs::JointState & js);

      /// Update the current robot state with the robot at pose.
      void updateRobotStatePose(const geometry_msgs::PoseStamped & pose);

      /// Get the markers for the current state.
      visualization_msgs::MarkerArray getMarkers(const std_msgs::ColorRGBA & color, const std::string & ns);

   private:
      bool currentStateInitialized() const;

   private:
      arm_navigation_msgs::RobotState _initState;       ///< the initialized state
      arm_navigation_msgs::RobotState _currentState;    ///< the current state after updates

      ros::ServiceClient _srvGetRobotMarker;
};

#endif

