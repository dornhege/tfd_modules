#ifndef ACTION_EXECUTOR_OPEN_DOOR_SERVICE_H
#define ACTION_EXECUTOR_OPEN_DOOR_SERVICE_H

#include "continual_planning_executive/actionExecutorService.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <std_srvs/Empty.h>

namespace tidyup_actions
{

    class ActionExecutorOpenDoorService : public ActionExecutorService<std_srvs::Empty>
    {
        public:
            virtual bool fillGoal(std_srvs::Empty::Request & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(bool success, std_srvs::Empty::Response & response,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

