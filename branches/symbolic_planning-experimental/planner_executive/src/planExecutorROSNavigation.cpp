#include "planExecutorROSNavigation.h"
#include "actionExecutorROSNavigation.h"

PlanExecutorROSNavigation::PlanExecutorROSNavigation(SymbolicState* current) : _currentState(current)
{
}

PlanExecutorROSNavigation::~PlanExecutorROSNavigation()
{
}

void PlanExecutorROSNavigation::createActionExecutors()
{
   _actionExecutors.push_back(new ActionExecutorROSNavigation(_currentState));
}

