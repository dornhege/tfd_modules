#ifndef NAVSTACK_MODULE_H
#define NAVSTACK_MODULE_H

#include "tfd_modules/module_api/pddlModuleTypes.h"

using namespace modules;

#ifdef __cplusplus
extern "C" {
#endif

void fullbody_navstack_init(int argc, char** argv);

double fullbody_pathCost(const ParameterList & parameterList, predicateCallbackType predicateCallback,
      numericalFluentCallbackType numericalFluentCallback, int relaxed);

VERIFY_CONDITIONCHECKER_DEF(fullbody_pathCost);

#ifdef __cplusplus
}
#endif


#endif

