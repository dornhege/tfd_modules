#ifndef ACTION_EXECUTOR_OBJECT_RECOGNITION_H
#define ACTION_EXECUTOR_OBJECT_RECOGNITION_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <tidyup_msgs/RecognizeObjectAction.h>

namespace tidyup_recognition_actions
{

    class ActionExecutorObjectRecognition : public ActionExecutorActionlib<tidyup_msgs::RecognizeObjectAction
                                        , tidyup_msgs::RecognizeObjectGoal, tidyup_msgs::RecognizeObjectResult>
    {
        public:
            virtual bool fillGoal(tidyup_msgs::RecognizeObjectGoal& goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const tidyup_msgs::RecognizeObjectResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif

