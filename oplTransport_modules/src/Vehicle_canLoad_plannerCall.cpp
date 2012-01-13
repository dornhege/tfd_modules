
#include <assert.h>
#include "State.h"
#include "Vehicle_canLoad_plannerCall.h"
#include "Vehicle_canLoad.h"

double Vehicle_canLoad_plannerCall(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback,
        modules::numericalFluentCallbackType numericalFluentCallback,
        int relaxed)
{
    assert(parameterList.size() == 2);
    // look up parameters in state
    opl::TransportModules::State* state = opl::TransportModules::State::stateInstance;
    const opl::TransportModules::Vehicle* this_pointer = state->getVehicles().find(parameterList.at(0).value)->second;
    const opl::TransportModules::Package* p = state->getPackages().find(parameterList.at(1).value)->second;

    // call lib interface
    bool value = opl::TransportModules::Vehicle_canLoad(state, this_pointer, p, relaxed);
    return (value ? 0.0 : modules::INFINITE_COST);
}


