#ifndef ACTION_EXECUTOR_TIDYUP_GRASP_OBJECT_H
#define ACTION_EXECUTOR_TIDYUP_GRASP_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <tidyup_msgs/PlaceObjectAction.h>

namespace tidyup_place_actions
{

    class ActionExecutorTidyupPlaceObject : public ActionExecutorActionlib<tidyup_msgs::PlaceObjectAction,
                                                    tidyup_msgs::PlaceObjectGoal, tidyup_msgs::PlaceObjectResult>
    {
        public:
            virtual bool fillGoal(tidyup_msgs::PlaceObjectGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState, const tidyup_msgs::PlaceObjectResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

