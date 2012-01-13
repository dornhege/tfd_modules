
#include "Vehicle.h"

namespace opl
{

namespace TransportModules
{

Vehicle::Vehicle(const std::string& name)
: Locatable(name)
{
}

Vehicle::~Vehicle()
{
}

void Vehicle::initialize()
{
    std::vector<std::string> readyLoadingArguments;
    readyLoadingArguments.push_back(getObjectID());

    std::string readyLoadingKey = opl::interface::ObjectLookupTable::instance->createKey("Vehicle_readyLoading", readyLoadingArguments);
    readyLoadingVariable = opl::interface::ObjectLookupTable::instance->getVariable(readyLoadingKey);
    std::vector<std::string> capacityArguments;
    capacityArguments.push_back(getObjectID());

    std::string capacityKey = opl::interface::ObjectLookupTable::instance->createKey("Vehicle_capacity", capacityArguments);
    capacityVariable = opl::interface::ObjectLookupTable::instance->getVariable(capacityKey);

}

bool Vehicle::readyLoading() const
{
    return opl::interface::ObjectLookupTable::instance->getPredicateValue(readyLoadingVariable);
}

double Vehicle::capacity() const
{
    return opl::interface::ObjectLookupTable::instance->getNumericValue(capacityVariable);
}


}

}
