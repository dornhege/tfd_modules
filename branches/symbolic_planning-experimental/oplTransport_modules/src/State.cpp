
#include "State.h"

namespace opl
{

namespace TransportModules
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

bool State::road(const opl::TransportModules::Location* l1, const opl::TransportModules::Location* l2) const
{
    const opl::interface::FluentMapping* roadVariable;
    std::vector<std::string> roadArguments;
    roadArguments.push_back(getObjectID());
    roadArguments.push_back(l1->getObjectID());
    roadArguments.push_back(l2->getObjectID());

    std::string roadKey = opl::interface::ObjectLookupTable::instance->createKey("road", roadArguments);
    roadVariable = opl::interface::ObjectLookupTable::instance->getVariable(roadKey);
    return opl::interface::ObjectLookupTable::instance->getPredicateValue(roadVariable);
}

double State::roadLength(const opl::TransportModules::Location* l1, const opl::TransportModules::Location* l2) const
{
    const opl::interface::FluentMapping* roadLengthVariable;
    std::vector<std::string> roadLengthArguments;
    roadLengthArguments.push_back(getObjectID());
    roadLengthArguments.push_back(l1->getObjectID());
    roadLengthArguments.push_back(l2->getObjectID());

    std::string roadLengthKey = opl::interface::ObjectLookupTable::instance->createKey("roadLength", roadLengthArguments);
    roadLengthVariable = opl::interface::ObjectLookupTable::instance->getVariable(roadLengthKey);
    return opl::interface::ObjectLookupTable::instance->getNumericValue(roadLengthVariable);
}


}

}
