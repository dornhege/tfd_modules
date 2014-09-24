#ifndef GOAL_CREATOR_FROM_STRING_H
#define GOAL_CREATOR_FROM_STRING_H

#include "continual_planning_executive/goalCreator.h"

namespace continual_planning_executive
{

class GoalCreatorFromString: public continual_planning_executive::GoalCreator
{
private:
    std::string goal_statement;
public:
    GoalCreatorFromString();
    ~GoalCreatorFromString();

    virtual void initialize(const std::deque<std::string> & arguments);
    virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
};

}
;

#endif

