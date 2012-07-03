#ifndef STATE_CREATOR_ROBOT_POSE_H
#define STATE_CREATOR_ROBOT_POSE_H

#include "continual_planning_executive/stateCreator.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include "tidyup_actions/robotPoseVisualization.h"

namespace tidyup_actions 
{

    /// This state creator adds the current robot pose to the state.
    /**
     * The current pose is estimated via tf as the transform from /map to /base_link.
     * A stamped pose in the state is represented by the fluents: x y z qx qy qz qw as well as timestamp and frame-id
     *
     * The state creator serves two purposes:
     * 1. The fluents for the robot_pose are set to the real values
     * 2. An at-style predicate of the form (at location) is set.
     *
     * 1. If _robotPoseObject is not empty the x,y,z,etc. fluents will be filled accordingly.
     * 2.a If the _atPredicate is not empty, the at predicate will be set to the current robot pose.
     * 2.b If a _locationType is given it is checked if the current robot pose is one of the
     *      poses in the objects of _locationType and thus _atPredicate is possibly set to
     *      a location instead of the current pose.
     *      If a robot is "at" a location is determined by the _goalToleranceXY and _goalToleranceYaw.
     */
    class StateCreatorRobotPose : public continual_planning_executive::StateCreator
    {
        public:
            StateCreatorRobotPose();
            ~StateCreatorRobotPose();

            /// Initialize the state creator parameters.
            /**
             * robot_pose robot_pose_type at_predicate locations_type
             *
             * If any parameter is given as "-" (dash), it is assumed empty.
             */
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillState(SymbolicState & state);

        protected:
            /// Extract a PoseStamped for object from state.
            /**
             * The fluents that are queried are: x,y,z, qx,qy,qz,qw, frame-id, timestamp
             *
             * \returns true if all fluents were available
             */
            bool extractPoseStamped(const SymbolicState & state, const string & object,
                    geometry_msgs::PoseStamped & pose) const;

            /// Get the color that location should have based on the fact that
            /// it is the robot location or another and if the robot is actually at that location.
            std_msgs::ColorRGBA getLocationColor(const SymbolicState & state, const string & location) const;

            /// Create a marker for location.
            visualization_msgs::MarkerArray getLocationMarkers(const SymbolicState & state, const string & location,
                    const string & ns, int id, bool useMeshes) const;

            void publishLocationsAsMarkers(const SymbolicState & state);

            /// Produces arm state for arms at side
            sensor_msgs::JointState getArmSideJointState() const;

        protected:
            tf::TransformListener _tf;

            double _goalToleranceXY;
            double _goalToleranceYaw;

            static const bool s_PublishLocationsAsMarkers = true;
            static const bool s_PublishMeshMarkers = true;

            ros::Publisher _markerPub;

            std::string _robotPoseObject;   ///< the name of the robot pose's object (e.g. robot_pose, or l0)
            std::string _robotPoseType;     ///< the type of the _robotPoseObject - required if _robotPoseObject.
            std::string _atPredicate;       ///< the name of the "at" predicate (e.g. at-base)
            std::string _locationType;      ///< the type of location objects that a robot might be "at"

            mutable RobotPoseVisualization _robotPoseVis;
    };

};

#endif

