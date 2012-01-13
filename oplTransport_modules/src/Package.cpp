
#include "Package.h"

namespace opl
{

namespace TransportModules
{

Package::Package(const std::string& name)
: Locatable(name)
{
}

Package::~Package()
{
}

void Package::initialize()
{
    std::vector<std::string> sizeArguments;
    sizeArguments.push_back(getObjectID());

    std::string sizeKey = opl::interface::ObjectLookupTable::instance->createKey("Package_size", sizeArguments);
    sizeVariable = opl::interface::ObjectLookupTable::instance->getVariable(sizeKey);

}

bool Package::in(const opl::TransportModules::Vehicle* v) const
{
    const opl::interface::FluentMapping* inVariable;
    std::vector<std::string> inArguments;
    inArguments.push_back(getObjectID());
    inArguments.push_back(v->getObjectID());

    std::string inKey = opl::interface::ObjectLookupTable::instance->createKey("Package_in", inArguments);
    inVariable = opl::interface::ObjectLookupTable::instance->getVariable(inKey);
    return opl::interface::ObjectLookupTable::instance->getPredicateValue(inVariable);
}

double Package::size() const
{
    return opl::interface::ObjectLookupTable::instance->getNumericValue(sizeVariable);
}


}

}
