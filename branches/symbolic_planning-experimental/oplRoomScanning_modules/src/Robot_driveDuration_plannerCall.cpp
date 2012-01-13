
#include <assert.h>
#include "State.h"
#include "Robot_driveDuration_plannerCall.h"
#include "Robot_driveDuration.h"

double driveDuration_plannerCall(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback,
        modules::numericalFluentCallbackType numericalFluentCallback,
        int relaxed)
{
    assert(parameterList.size() == 2);
    // look up parameters in state
    opl::RoomScanning::State* state = opl::RoomScanning::State::stateInstance;
    const opl::RoomScanning::Pose* origin = state->getPoses().find(parameterList.at(0).value)->second;
    const opl::RoomScanning::Pose* destination = state->getPoses().find(parameterList.at(1).value)->second;

    // call lib interface
    double value = driveDuration(state, origin, destination, relaxed);
    return value;
}


