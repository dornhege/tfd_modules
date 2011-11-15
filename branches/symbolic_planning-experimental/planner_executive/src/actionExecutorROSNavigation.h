#ifndef ACTION_EXECUTOR_R_O_S_NAVIGATION_H
#define ACTION_EXECUTOR_R_O_S_NAVIGATION_H

#include "continual_planning_executive/actionExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class ActionExecutorROSNavigation : public ActionExecutorInterface
{
   public:
      ActionExecutorROSNavigation(SymbolicState* current);
      ~ActionExecutorROSNavigation();

      virtual bool canExecute(const DurativeAction & a) const;

      virtual bool executeBlocking(const DurativeAction & a);

   protected:
      typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
      MoveBaseClient* _actionClient;

      SymbolicState* _currentState;
};

#endif

