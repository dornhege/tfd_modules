#ifndef STATE_CREATOR_OBJECT_RECOGNITION_H
#define STATE_CREATOR_OBJECT_RECOGNITION_H

#include "continual_planning_executive/stateCreator.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace tidyup_recognition_actions
{

    class StateCreatorObjectRecognition : public continual_planning_executive::StateCreator
    {
        public:
    		StateCreatorObjectRecognition();
            ~StateCreatorObjectRecognition();

            virtual bool fillState(SymbolicState & state);

        protected:
            tf::TransformListener _tf;

            double _goalTolerance;
    };

};

#endif

