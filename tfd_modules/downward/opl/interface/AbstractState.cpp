#include <stdio.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include "tfd_modules/opl/stringutil.h"
// HACK
#include "../../search/state.h"
#include "tfd_modules/opl/AbstractState.h"

namespace opl
{

namespace interface
{

inline bool double_equals(double a, double b)
{
    return std::abs(a - b) < 0.0001;
}

AbstractState::AbstractState()
{
    ObjectLookupTable::instance = this;
    currentState = NULL;
}

void AbstractState::updateFluentMappings()
{
    initialize();
    for (ObjectMappingsType::iterator objectIterator = objects.begin(); objectIterator != objects.end(); objectIterator++)
    {
        objectIterator->second->initialize();
    }
}

std::string AbstractState::createKey(const std::string& fluentName, const std::vector<std::string>& arguments) const
{
    std::string key = fluentName;
    for (std::vector<std::string>::const_iterator argumentIterator = arguments.begin(); argumentIterator != arguments.end(); argumentIterator++)
    {
        key += "%";
        key += *argumentIterator;
    }
//    std::cout << "AbstractState::createKey: " << StringUtil::toLower(key) << std::endl;
    return StringUtil::toLower(key);
}

std::string AbstractState::createObjectKey(int index, double value) const
{
    std::string key = StringUtil::createFromNumber(index);
    key += "|";
    key += StringUtil::createFromNumber((int)lrint(value));
    return key;
}

const FluentMapping* AbstractState::getVariable(const std::string& key) const
{
    return fluentMappings.find(key)->second;
}

bool AbstractState::getPredicateValue(const FluentMapping* variable) const
{
    if (variable == NULL)
    {
//        std::cout << " NULL fluent " << std::endl;
        return false;
    }
    else if (variable->isConstant())
    {
//        std::cout << " constant fluent " << (double_equals(0.0, variable->getValue())? "true":"false") << std::endl;
        return double_equals(0.0, variable->getValue());
    }
    else
    {
//        std::cout << " variable fluent " << variable->getValue() << " " << currentState->operator[](variable->getIndex()) << std::endl;
        return double_equals(currentState->operator[](variable->getIndex()), variable->getValue());
    }
}

double AbstractState::getNumericValue(const FluentMapping* variable) const
{
    if (variable == NULL)
    {
//        dump();
//        exit(1);
        return -0;
    }
    else if (variable->isConstant())
    {
        return variable->getValue();
    }
    else
    {
        return currentState->operator[](variable->getIndex());
    }
}

const Object* AbstractState::getObject(const FluentMapping* variable) const
{
    if (variable == NULL)
    {
        std::cout << "!! FluentMapping is NULL !!" << std::endl;
        return NULL;
    }
    else if (variable->isConstant())
    {
//        std::cout << " constant fluent " << std::endl;
        std::string objectKey = createObjectKey(variable->getIndex(), variable->getValue());
        return returnObjectMappings.find(objectKey)->second;
    }
    else
    {
//        std::cout << " variable fluent " << std::endl;
        std::string objectKey = createObjectKey(variable->getIndex(), currentState->operator[](variable->getIndex()));
        return returnObjectMappings.find(objectKey)->second;
    }
}

void AbstractState::dump() const
{
    std::cout << "OPL Callback Interface: State" << std::endl;
    std::cout << "\tObjects" << std::endl;
    for (ObjectMappingsType::const_iterator it = objects.begin(); it != objects.end(); it++)
    {
        std::cout << it->first << std::endl;
    }
    std::cout << "\tFluent mappings" << std::endl;
    for (FluentMappingsType::const_iterator it = fluentMappings.begin(); it != fluentMappings.end(); it++)
    {
        std::cout << it->first << " > " << it->second->dump();
        if (! it->second->isConstant())
        {
            std::cout << " in state: " << (*currentState)[it->second->getIndex()];
        }
        std::cout << std::endl;
    }
    std::cout << "\tObjects retrun mappings" << std::endl;
    for (ObjectMappingsType::const_iterator it = returnObjectMappings.begin(); it != returnObjectMappings.end(); it++)
    {
        std::cout << it->first << " > " << it->second->getObjectID() << std::endl;
    }

    std::cout << "\tTest" << std::endl;
//    std::cout << test() << std::endl;

}

}

}
