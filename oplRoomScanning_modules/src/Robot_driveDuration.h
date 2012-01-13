
#ifndef RoomScanning_Robot_driveDuration_plannerCall_H_
#define RoomScanning_Robot_driveDuration_plannerCall_H_

#include "State.h"

double driveDuration(
        const opl::RoomScanning::State* currentState,
        const opl::RoomScanning::Pose* origin, 
        const opl::RoomScanning::Pose* destination, 
        int relaxed);

#endif 
