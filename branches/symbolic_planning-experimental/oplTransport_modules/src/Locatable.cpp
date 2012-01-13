
#include "Locatable.h"

namespace opl
{

namespace TransportModules
{

Locatable::Locatable(const std::string& name)
: opl::interface::Object(name)
{
}

Locatable::~Locatable()
{
}

void Locatable::initialize()
{

}

bool Locatable::at(const opl::TransportModules::Location* l) const
{
    const opl::interface::FluentMapping* atVariable;
    std::vector<std::string> atArguments;
    atArguments.push_back(getObjectID());
    atArguments.push_back(l->getObjectID());

    std::string atKey = opl::interface::ObjectLookupTable::instance->createKey("Locatable_at", atArguments);
    atVariable = opl::interface::ObjectLookupTable::instance->getVariable(atKey);
    return opl::interface::ObjectLookupTable::instance->getPredicateValue(atVariable);
}


}

}
