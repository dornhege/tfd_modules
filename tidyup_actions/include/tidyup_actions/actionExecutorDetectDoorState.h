#ifndef ACTION_EXECUTOR_DETECT_DOOR_STATE_H
#define ACTION_EXECUTOR_DETECT_DOOR_STATE_H

#include "continual_planning_executive/actionExecutorService.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <tidyup_msgs/DetectDoorState.h>

namespace tidyup_actions
{

    class ActionExecutorDetectDoorState : public ActionExecutorService<tidyup_msgs::DetectDoorState>
    {
        public:
            virtual bool fillGoal(tidyup_msgs::DetectDoorState::Request & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(bool success, tidyup_msgs::DetectDoorState::Response & response,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

