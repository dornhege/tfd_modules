#ifndef PLAN_EXECUTOR_R_O_S_NAVIGATION_H
#define PLAN_EXECUTOR_R_O_S_NAVIGATION_H

#include "planExecutorInterface.h"
#include "continual_planning_executive/symbolicState.h"

class PlanExecutorROSNavigation : public PlanExecutorInterface
{
   public:
      PlanExecutorROSNavigation(SymbolicState* current);
      ~PlanExecutorROSNavigation();

      virtual void createActionExecutors();

   protected:
      SymbolicState* _currentState;
};

#endif

