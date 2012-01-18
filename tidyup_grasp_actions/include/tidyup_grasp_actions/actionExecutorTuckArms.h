#ifndef ACTION_EXECUTOR_TUCK_ARMS_H
#define ACTION_EXECUTOR_TUCK_ARMS_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <pr2_common_action_msgs/TuckArmsAction.h>

namespace tidyup_grasp_actions
{

    class ActionExecutorTuckArms : public ActionExecutorActionlib<pr2_common_action_msgs::TuckArmsAction,
                                                    pr2_common_action_msgs::TuckArmsGoal, pr2_common_action_msgs::TuckArmsResult>
    {
        public:
            virtual bool fillGoal(pr2_common_action_msgs::TuckArmsGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState, const pr2_common_action_msgs::TuckArmsResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

