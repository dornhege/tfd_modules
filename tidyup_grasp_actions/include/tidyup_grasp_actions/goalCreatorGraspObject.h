#ifndef GOAL_CREATOR_GRASP_OBJECT_H
#define GOAL_CREATOR_GRASP_OBJECT_H

#include "continual_planning_executive/goalCreator.h"

namespace tidyup_grasp_actions 
{

    class GoalCreatorGraspObject : public continual_planning_executive::GoalCreator
    {
        public:
            GoalCreatorGraspObject();
            ~GoalCreatorGraspObject();

            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
    };

};

#endif

