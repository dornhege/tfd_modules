#include "crewPlanning.h"
#include <stdlib.h>
#include <sys/time.h>
#include <ros/ros.h>

double available(const ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback,
      int relaxed, plannerContextPtr context, plannerContextCompareType contextComp, bool & tookContext)
{
  PredicateList* list = new PredicateList();
  list->push_back(Predicate("available",parameterList));
  predicateCallback(list);
  if(list->front().value)
     return 0;
  else
     return INFINITE_COST;
}

int effectCall(const ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback,
      vector<double>& writtenVars, plannerContextPtr context, plannerContextCompareType contextComp, bool & tookContext)
{
   tookContext = false;
   NumericalFluentList* list = new NumericalFluentList();
   list->push_back(NumericalFluent("effectscalled",parameterList)); // lower case as PDDL is case insensitive
   numericalFluentCallback(list);
   writtenVars[0] = (list->front().value + 1.0);
   return 1;
}

