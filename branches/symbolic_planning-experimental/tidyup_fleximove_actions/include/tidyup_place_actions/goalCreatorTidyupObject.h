#ifndef GOAL_CREATOR_GRASP_OBJECT_H
#define GOAL_CREATOR_GRASP_OBJECT_H

#include "continual_planning_executive/goalCreator.h"

namespace tidyup_place_actions 
{

    class GoalCreatorTidyupObject : public continual_planning_executive::GoalCreator
    {
        public:
            GoalCreatorTidyupObject();
            ~GoalCreatorTidyupObject();

            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
    };

};

#endif

