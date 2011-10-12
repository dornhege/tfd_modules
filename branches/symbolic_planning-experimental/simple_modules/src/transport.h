#ifndef TRANSPORT_MODULE_H
#define TRANSPORT_MODULE_H

#include "tfd_modules/module_api/pddlModuleTypes.h"

using namespace modules;

#ifdef __cplusplus
extern "C" {
#endif

void init(int argc, char** argv);

double can_load(const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed);

VERIFY_CONDITIONCHECKER_DEF(can_load);

//int effectCall(ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback,
  //    vector<double>& writtenVars);

subplanType genSubplan(const string & operatorName, const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int heuristic);
VERIFY_SUBPLANGENERATOR_DEF(genSubplan);

/// For final plan output: Convert a subplan into a string.
string outputSubplan(subplanType sp);

void execSubplan(modulePlanType modulePlan);


#ifdef __cplusplus
}
#endif

#endif

