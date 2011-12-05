#ifndef ACTION_EXECUTOR_REQUEST_GRASPABLE_OBJECTS_H
#define ACTION_EXECUTOR_REQUEST_GRASPABLE_OBJECTS_H

#include "continual_planning_executive/actionExecutorService.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <tidyup_msgs/RequestGraspableObjects.h>

namespace tidyup_grasp_actions
{

    class ActionExecutorRequestGraspableObjects : public ActionExecutorService<tidyup_msgs::RequestGraspableObjects>
    {
        public:
            virtual bool fillGoal(tidyup_msgs::RequestGraspableObjects::Request & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(bool success, tidyup_msgs::RequestGraspableObjects::Response & response,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

