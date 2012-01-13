
#include "Door.h"

namespace opl
{

namespace RoomScanning
{

Door::Door(const std::string& name)
: opl::interface::Object(name)
{
}

Door::~Door()
{
}

void Door::initialize()
{
    opl::interface::Object::initialize();
    std::vector<std::string> approachPoseArguments;
    approachPoseArguments.push_back(getObjectID());

    std::string approachPoseKey = opl::interface::ObjectLookupTable::instance->createKey("Door_approachPose", approachPoseArguments);
    approachPoseVariable = opl::interface::ObjectLookupTable::instance->getVariable(approachPoseKey);
    std::vector<std::string> openArguments;
    openArguments.push_back(getObjectID());

    std::string openKey = opl::interface::ObjectLookupTable::instance->createKey("Door_open", openArguments);
    openVariable = opl::interface::ObjectLookupTable::instance->getVariable(openKey);

}

const opl::RoomScanning::Pose* Door::approachPose() const
{
    return dynamic_cast<const opl::RoomScanning::Pose*>(opl::interface::ObjectLookupTable::instance->getObject(approachPoseVariable));
}

bool Door::open() const
{
    return opl::interface::ObjectLookupTable::instance->getPredicateValue(openVariable);
}


}

}
