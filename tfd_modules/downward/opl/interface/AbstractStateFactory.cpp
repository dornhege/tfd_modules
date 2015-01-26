/*
 * AbstractStateFactory.cpp
 *
 *  Created on: Jul 20, 2011
 *      Author: Andreas Hertle
 */

#include <assert.h>
#include "tfd_modules/opl/stringutil.h"
#include "tfd_modules/opl/AbstractStateFactory.h"

namespace opl
{

namespace interface
{

opl::interface::AbstractState* AbstractStateFactory::createState(
        const modules::ObjectTypeMap& objects,
        const modules::PredicateMapping& predicateMapping,
        const modules::FunctionMapping& functionMapping,
        const modules::PredicateList& predicateConstants,
        const modules::NumericalFluentList& numericConstants)
{
    AbstractState* state = instantiateState(objects);

    // create fluents mappings
    for (modules::PredicateMapping::const_iterator fluentIterator = predicateMapping.begin(); fluentIterator != predicateMapping.end(); fluentIterator++)
    {
        createFluentMapping(state, *fluentIterator);
    }
    for (modules::FunctionMapping::const_iterator fluentIterator = functionMapping.begin(); fluentIterator != functionMapping.end(); fluentIterator++)
    {
        createFluentMapping(state, *fluentIterator);
    }

    // create constants mappings
    for (modules::PredicateList::const_iterator fluentIterator = predicateConstants.begin(); fluentIterator != predicateConstants.end(); fluentIterator++)
    {
        createFluentMapping(state, *fluentIterator);
    }
    for (modules::NumericalFluentList::const_iterator fluentIterator = numericConstants.begin(); fluentIterator != numericConstants.end(); fluentIterator++)
    {
        createFluentMapping(state, *fluentIterator);
    }

    state->updateFluentMappings();

    return state;
}

void AbstractStateFactory::createFluentMapping(AbstractState* state, const std::pair<const std::string, modules::VarVal>& mapping)
{
    vector<string> arguments = StringUtil::split(mapping.first, " ");
    std::string fluentName = arguments.front();
    arguments.erase(arguments.begin());

    std::string::size_type index = fluentName.find_first_of("!");
    if (index != std::string::npos)
    {
        // Object fluent: create return object mapping
        string realFluentName = fluentName.substr(0, index);
        fluentName = realFluentName;
        std::string returnObjectName = arguments.back();
        arguments.pop_back();
        opl::interface::Object* returnObject = state->objects.find(returnObjectName)->second;
        std::string returnObjectKey = ObjectLookupTable::instance->createObjectKey(mapping.second.first, mapping.second.second);
        state->returnObjectMappings.insert(make_pair(returnObjectKey, returnObject));
    }

    // create key and FluentMapping
    std::string key = opl::interface::ObjectLookupTable::instance->createKey(fluentName, arguments);
    opl::interface::FluentMapping* value = new opl::interface::FluentMapping(mapping.second.first, mapping.second.second);

    // initialize fluent
    state->fluentMappings.insert(make_pair(key, value));
}

void AbstractStateFactory::createFluentMapping(AbstractState* state, const std::pair<const std::string, int>& mapping)
{
    vector<string> arguments = StringUtil::split(mapping.first, " ");
    std::string fluentName = arguments.front();
    arguments.erase(arguments.begin());

    // create key and FluentMapping
    std::string key = opl::interface::ObjectLookupTable::instance->createKey(fluentName, arguments);
    opl::interface::FluentMapping* value = new opl::interface::FluentMapping(mapping.second);

    // initialize fluent
    state->fluentMappings.insert(make_pair(key, value));
}

void AbstractStateFactory::createFluentMapping(AbstractState* state, modules::Predicate fluent)
{
    modules::ParameterList parameters = fluent.parameters;
    vector<string> arguments;
    for (modules::ParameterList::const_iterator parameterIterator = parameters.begin(); parameterIterator != parameters.end(); parameterIterator++)
    {
        arguments.push_back(parameterIterator->value);
    }
    std::string fluentName = fluent.name;
    std::string::size_type index = fluentName.find_first_of("!");
    if (index != std::string::npos)
    {
        // Object fluent: create return object mapping
        string realFluentName = fluentName.substr(0, index);
        fluentName = realFluentName;
        std::string returnObjectName = arguments.back();
        arguments.pop_back();
        opl::interface::Object* returnObject = state->objects.find(returnObjectName)->second;
        std::string returnObjectKey = ObjectLookupTable::instance->createObjectKey((int)state->fluentMappings.size(), 0);
        state->returnObjectMappings.insert(make_pair(returnObjectKey, returnObject));
    }

    // create key and FluentMapping
    std::string key = opl::interface::ObjectLookupTable::instance->createKey(fluentName, arguments);
    opl::interface::FluentMapping* value = new opl::interface::FluentMapping((int)state->fluentMappings.size(), 0, true);

    // initialize fluent
    state->fluentMappings.insert(make_pair(key, value));
}

void AbstractStateFactory::createFluentMapping(AbstractState* state, const modules::NumericalFluent& fluent)
{
    modules::ParameterList parameters = fluent.parameters;
    vector<string> arguments;
    for (modules::ParameterList::const_iterator parameterIterator = parameters.begin(); parameterIterator != parameters.end(); parameterIterator++)
    {
        arguments.push_back(parameterIterator->value);
    }
    std::string fluentName = fluent.name;

    // create key and FluentMapping
    std::string key = opl::interface::ObjectLookupTable::instance->createKey(fluentName, arguments);
    opl::interface::FluentMapping* value = new opl::interface::FluentMapping((int)state->fluentMappings.size(), fluent.value, true);

    // initialize fluent
    state->fluentMappings.insert(make_pair(key, value));
}

}

}


