#ifndef ACTION_EXECUTOR_TUCK_ARMS_H
#define ACTION_EXECUTOR_TUCK_ARMS_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <pr2_python_services/ArmToSideAction.h>

namespace tidyup_place_actions
{

    class ActionExecutorArmToSide : public ActionExecutorActionlib<pr2_python_services::ArmToSideAction,
    pr2_python_services::ArmToSideGoal, pr2_python_services::ArmToSideResult>
    {
        public:
            virtual bool fillGoal(pr2_python_services::ArmToSideGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState, const pr2_python_services::ArmToSideResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

