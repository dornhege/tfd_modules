#ifndef STATE_CREATOR_ROBOT_LOCATION_IN_ROOM_H
#define STATE_CREATOR_ROBOT_LOCATION_IN_ROOM_H

#include "continual_planning_executive/stateCreator.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

namespace tidyup_actions 
{

    /// This state creator infers the room the current robot pose is in, if that is not set.
    /**
     * This is usually only needed for initialization.
     */
    class StateCreatorRobotLocationInRoom : public continual_planning_executive::StateCreator
    {
        public:
            StateCreatorRobotLocationInRoom();
            ~StateCreatorRobotLocationInRoom();

            /// Initialize the state creator parameters.
            /**
             * robot_pose locations_type
             */
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillState(SymbolicState & state);

        protected:
            tf::TransformListener _tf;

            std::string _robotPoseObject;   ///< the name of the robot pose's object (e.g. robot_pose, or l0)
            std::string _locationType;      ///< the type of location objects that a robot might be "at"
    };

};

#endif

