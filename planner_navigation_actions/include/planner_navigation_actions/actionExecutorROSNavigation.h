#ifndef ACTION_EXECUTOR_R_O_S_NAVIGATION_H
#define ACTION_EXECUTOR_R_O_S_NAVIGATION_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace planner_navigation_actions
{

    class ActionExecutorROSNavigation : public continual_planning_executive::ActionExecutorInterface
    {
        public:
            ActionExecutorROSNavigation();
            ~ActionExecutorROSNavigation();

            virtual bool canExecute(const DurativeAction & a, const SymbolicState & current) const;

            virtual bool executeBlocking(const DurativeAction & a, SymbolicState & current);

        protected:
            typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
            MoveBaseClient* _actionClient;
    };

};

#endif

