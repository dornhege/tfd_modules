
#ifndef RoomScanning_Robot_driveDuration_H_
#define RoomScanning_Robot_driveDuration_H_

#include "tfd_modules/module_api/pddlModuleTypes.h"

using namespace modules;

#ifdef __cplusplus
extern "C" {
#endif

void navstack_init(int argc, char** argv);

double driveDuration_plannerCall(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback,
        modules::numericalFluentCallbackType numericalFluentCallback,
        int relaxed);

#ifdef __cplusplus
}
#endif

#endif 
