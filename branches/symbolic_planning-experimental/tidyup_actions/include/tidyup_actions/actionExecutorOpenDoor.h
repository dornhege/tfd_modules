#ifndef ACTION_EXECUTOR_OPEN_DOOR_H
#define ACTION_EXECUTOR_OPEN_DOOR_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <tidyup_msgs/OpenDoorAction.h>

namespace tidyup_actions
{

    class ActionExecutorOpenDoor : public ActionExecutorActionlib<tidyup_msgs::OpenDoorAction,
                                                    tidyup_msgs::OpenDoorGoal, tidyup_msgs::OpenDoorResult>
    {
        public:
            virtual bool fillGoal(tidyup_msgs::OpenDoorGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const tidyup_msgs::OpenDoorResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

