#ifndef GOAL_CREATOR_H
#define GOAL_CREATOR_H

#include "symbolicState.h"
#include <stdio.h>
#include <string>
using std::string;

namespace continual_planning_executive
{

    /// Base interface for creating goals as SymbolicStates from the world.
    /**
     * GoalCreators are specific to a certain domain.
     * There can be multiple GoalCreators that fill different aspects of the symbolic state.
     * They should not contradict each other.
     */
    class GoalCreator
    {
        public:
            GoalCreator() {}
            virtual ~GoalCreator() {}

            /// Create the goal by filling the goal state, certain aspects for the goal might be added to the current state.
            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goalState) = 0;
    };

};

#endif

