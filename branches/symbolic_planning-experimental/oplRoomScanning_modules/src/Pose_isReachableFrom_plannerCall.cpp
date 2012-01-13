
#include <assert.h>
#include "State.h"
#include "Pose_isReachableFrom_plannerCall.h"
#include "Pose_isReachableFrom.h"

double Pose_isReachableFrom_plannerCall(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback,
        modules::numericalFluentCallbackType numericalFluentCallback,
        int relaxed)
{
    assert(parameterList.size() == 2);
    // look up parameters in state
    opl::RoomScanning::State* state = opl::RoomScanning::State::stateInstance;
    const opl::RoomScanning::Pose* this_pointer = state->getPoses().find(parameterList.at(0).value)->second;
    const opl::RoomScanning::Pose* origin = state->getPoses().find(parameterList.at(1).value)->second;

    // call lib interface
    bool value = opl::RoomScanning::Pose_isReachableFrom(state, this_pointer, origin, relaxed);
    return (value ? 0.0 : modules::INFINITE_COST);
}


