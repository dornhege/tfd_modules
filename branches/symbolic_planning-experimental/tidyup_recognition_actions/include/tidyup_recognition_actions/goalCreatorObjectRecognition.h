#ifndef GOAL_CREATOR_OBJECT_RECOGNITION_H
#define GOAL_CREATOR_OBJECT_RECOGNITION_H

#include "continual_planning_executive/goalCreator.h"

namespace tidyup_recognition_actions
{

    class GoalCreatorObjectRecognition : public continual_planning_executive::GoalCreator
    {
        public:
    		GoalCreatorObjectRecognition();
            ~GoalCreatorObjectRecognition();

            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
    };

};

#endif

