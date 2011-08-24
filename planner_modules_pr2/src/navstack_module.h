#ifndef NAVSTACK_MODULE_H
#define NAVSTACK_MODULE_H

#include "tfd_modules/module_api/pddlModuleTypes.h"

using namespace modules;

#ifdef __cplusplus
extern "C" {
#endif

void navstack_init(int argc, char** argv);

double pathCost(const ParameterList & parameterList, predicateCallbackType predicateCallback, 
      numericalFluentCallbackType numericalFluentCallback,
      int relaxed, plannerContextPtr context, plannerContextCompareType contextComp, bool & tookContext);

VERIFY_CONDITIONCHECKER_DEF(pathCost);

#ifdef __cplusplus
}
#endif


#endif

