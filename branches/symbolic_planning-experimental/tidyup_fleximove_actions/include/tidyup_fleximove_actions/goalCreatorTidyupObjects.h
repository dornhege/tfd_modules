#ifndef GOAL_CREATOR_GRASP_OBJECT_H
#define GOAL_CREATOR_GRASP_OBJECT_H

#include "continual_planning_executive/goalCreator.h"

namespace tidyup_fleximove_actions
{

    class GoalCreatorTidyupObjects : public continual_planning_executive::GoalCreator
    {
        public:
            GoalCreatorTidyupObjects();
            ~GoalCreatorTidyupObjects();

            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
    };

};

#endif

