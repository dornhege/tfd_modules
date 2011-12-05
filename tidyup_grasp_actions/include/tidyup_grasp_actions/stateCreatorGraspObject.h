#ifndef STATE_CREATOR_GRASP_OBJECT_H
#define STATE_CREATOR_GRASP_OBJECT_H

#include "continual_planning_executive/stateCreator.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace tidyup_grasp_actions 
{

    class StateCreatorGraspObject : public continual_planning_executive::StateCreator
    {
        public:
            StateCreatorGraspObject();
            ~StateCreatorGraspObject();

            virtual bool fillState(SymbolicState & state);

        protected:
            tf::TransformListener _tf;

            double _goalTolerance;
    };

};

#endif

