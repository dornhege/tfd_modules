#ifndef STATE_CREATOR_GRASP_OBJECT_H
#define STATE_CREATOR_GRASP_OBJECT_H

#include "continual_planning_executive/stateCreator.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

namespace tidyup_place_actions 
{

    class StateCreatorTidyupObject : public continual_planning_executive::StateCreator
    {
        public:
            StateCreatorTidyupObject();
            ~StateCreatorTidyupObject();

            virtual bool fillState(SymbolicState & state);

        protected:
            void publishLocationsAsMarkers(const SymbolicState & state);

        protected:
            tf::TransformListener _tf;

            double _goalToleranceXY;
            double _goalToleranceYaw;

            static const bool s_PublishLocationsAsMarkers = true;
            ros::Publisher _markerPub;
    };

};

#endif

