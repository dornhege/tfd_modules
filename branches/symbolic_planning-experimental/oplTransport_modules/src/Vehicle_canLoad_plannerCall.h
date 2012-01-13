
#ifndef TransportModules_Vehicle_canLoad_H_
#define TransportModules_Vehicle_canLoad_H_

#ifdef __cplusplus
extern "C" {
#endif

double Vehicle_canLoad_plannerCall(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback,
        modules::numericalFluentCallbackType numericalFluentCallback,
        int relaxed);

#ifdef __cplusplus
}
#endif

#endif 
