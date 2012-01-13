
#include "Robot.h"

namespace opl
{

namespace RoomScanning
{

Robot::Robot(const std::string& name)
: opl::interface::Object(name)
{
}

Robot::~Robot()
{
}

void Robot::initialize()
{
    opl::interface::Object::initialize();
    std::vector<std::string> currentPoseArguments;
    currentPoseArguments.push_back(getObjectID());

    std::string currentPoseKey = opl::interface::ObjectLookupTable::instance->createKey("Robot_currentPose", currentPoseArguments);
    currentPoseVariable = opl::interface::ObjectLookupTable::instance->getVariable(currentPoseKey);
    std::vector<std::string> busyArguments;
    busyArguments.push_back(getObjectID());

    std::string busyKey = opl::interface::ObjectLookupTable::instance->createKey("Robot_busy", busyArguments);
    busyVariable = opl::interface::ObjectLookupTable::instance->getVariable(busyKey);

}

const opl::RoomScanning::Pose* Robot::currentPose() const
{
    return dynamic_cast<const opl::RoomScanning::Pose*>(opl::interface::ObjectLookupTable::instance->getObject(currentPoseVariable));
}

bool Robot::busy() const
{
    return opl::interface::ObjectLookupTable::instance->getPredicateValue(busyVariable);
}

const opl::RoomScanning::Pose* Robot::middle(const opl::RoomScanning::Pose* p1, const opl::RoomScanning::Pose* p2) const
{
    const opl::interface::FluentMapping* middleVariable;
    std::vector<std::string> middleArguments;
    middleArguments.push_back(getObjectID());
    middleArguments.push_back(p1->getObjectID());
    middleArguments.push_back(p2->getObjectID());

    std::string middleKey = opl::interface::ObjectLookupTable::instance->createKey("Robot_middle", middleArguments);
    middleVariable = opl::interface::ObjectLookupTable::instance->getVariable(middleKey);
    return dynamic_cast<const opl::RoomScanning::Pose*>(opl::interface::ObjectLookupTable::instance->getObject(middleVariable));
}


}

}
