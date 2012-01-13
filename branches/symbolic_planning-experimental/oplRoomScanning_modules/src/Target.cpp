
#include "Target.h"

namespace opl
{

namespace RoomScanning
{

Target::Target(const std::string& name)
: Pose(name)
{
}

Target::~Target()
{
}

void Target::initialize()
{
    Pose::initialize();
    std::vector<std::string> exploredArguments;
    exploredArguments.push_back(getObjectID());

    std::string exploredKey = opl::interface::ObjectLookupTable::instance->createKey("Target_explored", exploredArguments);
    exploredVariable = opl::interface::ObjectLookupTable::instance->getVariable(exploredKey);

}

bool Target::explored() const
{
    return opl::interface::ObjectLookupTable::instance->getPredicateValue(exploredVariable);
}


}

}
