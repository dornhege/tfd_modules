#ifndef ACTION_EXECUTOR_DETECT_DOOR_STATE_ANGLE_H
#define ACTION_EXECUTOR_DETECT_DOOR_STATE_ANGLE_H

#include "continual_planning_executive/actionExecutorService.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <door_msgs/DoorStateSrv.h>

namespace tidyup_actions
{

   class ActionExecutorDetectDoorStateAngle : public ActionExecutorService<door_msgs::DoorStateSrv>
    {
        public:
            virtual void initialize(const std::deque<std::string> & arguments);
            virtual bool fillGoal(door_msgs::DoorStateSrv::Request & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(bool success, door_msgs::DoorStateSrv::Response & response,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

