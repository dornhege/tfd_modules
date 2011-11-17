#ifndef GOAL_CREATOR_R_O_S_NAVIGATION_H
#define GOAL_CREATOR_R_O_S_NAVIGATION_H

#include "continual_planning_executive/goalCreator.h"

namespace planner_navigation_actions
{

    class GoalCreatorROSNavigation : public continual_planning_executive::GoalCreator
    {
        public:
            GoalCreatorROSNavigation();
            ~GoalCreatorROSNavigation();

            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
    };

};

#endif

