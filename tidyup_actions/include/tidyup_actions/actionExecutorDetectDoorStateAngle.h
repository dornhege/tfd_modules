#ifndef ACTION_EXECUTOR_DETECT_DOOR_STATE_ANGLE_H
#define ACTION_EXECUTOR_DETECT_DOOR_STATE_ANGLE_H

#include "continual_planning_executive/actionExecutorService.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <tidyup_msgs/DoorState.h>

namespace tidyup_actions
{

   class ActionExecutorDetectDoorStateAngle : public ActionExecutorService<tidyup_msgs::DoorState>
    {
        public:
            virtual void initialize(const std::deque<std::string> & arguments);
            virtual bool fillGoal(tidyup_msgs::DoorState::Request & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(bool success, tidyup_msgs::DoorState::Response & response,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

