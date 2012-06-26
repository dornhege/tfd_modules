#ifndef ACTION_EXECUTOR_TUCK_ARMS_H
#define ACTION_EXECUTOR_TUCK_ARMS_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <tidyup_msgs/ArmToSideAction.h>

namespace tidyup_place_actions
{

    class ActionExecutorArmToSide : public ActionExecutorActionlib<tidyup_msgs::ArmToSideAction,
    tidyup_msgs::ArmToSideGoal, tidyup_msgs::ArmToSideResult>
    {
        public:
            virtual bool fillGoal(tidyup_msgs::ArmToSideGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState, const tidyup_msgs::ArmToSideResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

