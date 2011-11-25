#ifndef ACTION_EXECUTOR_R_O_S_NAVIGATION_H
#define ACTION_EXECUTOR_R_O_S_NAVIGATION_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <move_base_msgs/MoveBaseAction.h>

namespace planner_navigation_actions
{

    class ActionExecutorROSNavigation : public ActionExecutorActionlib
                                        <move_base_msgs::MoveBaseAction, move_base_msgs::MoveBaseGoal>
    {
        public:
            virtual bool fillGoal(move_base_msgs::MoveBaseGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

