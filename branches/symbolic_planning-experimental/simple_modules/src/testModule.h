#ifndef TEST_MODULE_H
#define TEST_MODULE_H

#include "tfd_modules/module_api/pddlModuleTypes.h"

using namespace modules;

#ifdef __cplusplus
extern "C" {
#endif

/// These three are always true/false or 50:50
double checkTrue(const ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed);
double checkFalse(const ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed);
double checkRandom(const ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed);

VERIFY_CONDITIONCHECKER_DEF(checkTrue);
VERIFY_CONDITIONCHECKER_DEF(checkFalse);
VERIFY_CONDITIONCHECKER_DEF(checkRandom);

/// Basically: if 1st parameter has equal chars (e.g. car1) it's true - just to test param passing
double checkParamEqualCharacterCount(const ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed);

VERIFY_CONDITIONCHECKER_DEF(checkParamEqualCharacterCount);

/// will return stepped-on fluent + 1
int dummyEffect(const ParameterList & parameterList,
          predicateCallbackType  predicateCallback,
          numericalFluentCallbackType numericalFluentCallback,
	 vector<double>& writtenVars);

int countIt(const ParameterList & parameterList,
          predicateCallbackType  predicateCallback,
          numericalFluentCallbackType numericalFluentCallback,
	 vector<double>& writtenVars);

VERIFY_APPLYEFFECT_DEF(dummyEffect);
VERIFY_APPLYEFFECT_DEF(countIt);

// simple cost module for 3 locations l1, l2, l3.
double costDrive(const ParameterList & parameterList, predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed);

VERIFY_CONDITIONCHECKER_DEF(costDrive);

#ifdef __cplusplus
}
#endif

#endif

