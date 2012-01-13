
#include "Location.h"

namespace opl
{

namespace TransportModules
{

Location::Location(const std::string& name)
: opl::interface::Object(name)
{
}

Location::~Location()
{
}

void Location::initialize()
{
    std::vector<std::string> hasPetrolStationArguments;
    hasPetrolStationArguments.push_back(getObjectID());

    std::string hasPetrolStationKey = opl::interface::ObjectLookupTable::instance->createKey("Location_hasPetrolStation", hasPetrolStationArguments);
    hasPetrolStationVariable = opl::interface::ObjectLookupTable::instance->getVariable(hasPetrolStationKey);

}

bool Location::hasPetrolStation() const
{
    return opl::interface::ObjectLookupTable::instance->getPredicateValue(hasPetrolStationVariable);
}


}

}
