#ifndef ACTION_EXECUTOR_POST_GRASP_POSITION_H
#define ACTION_EXECUTOR_POST_GRASP_POSITION_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <tidyup_msgs/PostGraspPositionAction.h>

namespace tidyup_grasp_actions
{

    class ActionExecutorPostGraspPosition : public ActionExecutorActionlib<tidyup_msgs::PostGraspPositionAction,
                                                    tidyup_msgs::PostGraspPositionGoal, tidyup_msgs::PostGraspPositionResult>
    {
        public:
            virtual bool fillGoal(tidyup_msgs::PostGraspPositionGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState, const tidyup_msgs::PostGraspPositionResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

