
#include "Pose_isReachableFrom.h"

namespace opl
{

namespace RoomScanning
{

bool Pose_isReachableFrom(const State* currentState,
        const opl::RoomScanning::Pose* this_pointer,
        const opl::RoomScanning::Pose* origin,
        int relaxed)
{
    const std::map<std::string, Door*>& doors = currentState->getDoors();
    double maxReachableX = 20;
    for (std::map<std::string, Door*>::const_iterator doorIt = doors.begin(); doorIt != doors.end(); doorIt++)
    {
        const Door* door = doorIt->second;
        if (! door->open())
        {
            maxReachableX = fmin(maxReachableX, door->approachPose()->x());
        }
    }
    if (this_pointer->x() <= maxReachableX)
    {
        return true;
    }
    return false;
}

}

}

