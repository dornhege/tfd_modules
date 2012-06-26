#ifndef ACTION_EXECUTOR_GRASP_OBJECT_H
#define ACTION_EXECUTOR_GRASP_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <tidyup_msgs/GraspObjectAction.h>

namespace tidyup_actions
{

    class ActionExecutorPickupObject : public ActionExecutorActionlib<tidyup_msgs::GraspObjectAction,
                                                    tidyup_msgs::GraspObjectGoal, tidyup_msgs::GraspObjectResult>
    {
        public:
            virtual bool fillGoal(tidyup_msgs::GraspObjectGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const tidyup_msgs::GraspObjectResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

