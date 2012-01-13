
#ifndef RoomScanning_Pose_isReachableFrom_plannerCall_H_
#define RoomScanning_Pose_isReachableFrom_plannerCall_H_

#include "State.h"

namespace opl
{

namespace RoomScanning
{

bool Pose_isReachableFrom(const State* currentState,
        const opl::RoomScanning::Pose* this_pointer, 
        const opl::RoomScanning::Pose* origin, 
        int relaxed);

}

}

#endif 
