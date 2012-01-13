
#include "State.h"

namespace opl
{

namespace RoomScanning
{
    
State* State::stateInstance = NULL;

State::State()
{
    State::stateInstance = this;
}

State::~State()
{
}

void State::initialize()
{

}

bool State::inRange(const opl::RoomScanning::Pose* p1, const opl::RoomScanning::Pose* p2) const
{
    const opl::interface::FluentMapping* inRangeVariable;
    std::vector<std::string> inRangeArguments;
    inRangeArguments.push_back(getObjectID());
    inRangeArguments.push_back(p1->getObjectID());
    inRangeArguments.push_back(p2->getObjectID());

    std::string inRangeKey = opl::interface::ObjectLookupTable::instance->createKey("inRange", inRangeArguments);
    inRangeVariable = opl::interface::ObjectLookupTable::instance->getVariable(inRangeKey);
    return opl::interface::ObjectLookupTable::instance->getPredicateValue(inRangeVariable);
}


}

}
