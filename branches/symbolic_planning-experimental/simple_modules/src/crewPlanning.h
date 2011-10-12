#ifndef TEST_MODULE_H
#define TEST_MODULE_H

#include "tfd_modules/module_api/pddlModuleTypes.h"

using namespace modules;

#ifdef __cplusplus
extern "C" {
#endif

/// These three are always true/false or 50:50
double available(const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed);

int effectCall(const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback,
        vector<double>& writtenVars);

VERIFY_CONDITIONCHECKER_DEF(available);
VERIFY_APPLYEFFECT_DEF(effectCall);

#ifdef __cplusplus
}
#endif

#endif

