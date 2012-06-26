#ifndef ACTION_EXECUTOR_MOVE_BASE_H
#define ACTION_EXECUTOR_MOVE_BASE_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <move_base_msgs/MoveBaseAction.h>

namespace tidyup_actions
{

    class ActionExecutorMoveBase : public ActionExecutorActionlib<move_base_msgs::MoveBaseAction,
                                                    move_base_msgs::MoveBaseGoal, move_base_msgs::MoveBaseResult>
    {
        public:
            virtual bool fillGoal(move_base_msgs::MoveBaseGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const move_base_msgs::MoveBaseResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

