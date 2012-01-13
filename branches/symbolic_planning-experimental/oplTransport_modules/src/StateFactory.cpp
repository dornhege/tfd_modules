
#include <assert.h>
#include <iostream>
#include "tfd_modules/opl/stringutil.h"
#include "StateFactory.h"

opl::interface::OplCallbackInterface* initCallbackInterface(
        const ObjectStringList& objects,
        const PredicateMapping& predicateMapping,
        const FunctionMapping& functionMapping,
        const modules::PredicateList& predicateConstants,
        const modules::NumericalFluentList& numericConstants)
{
    opl::interface::AbstractStateFactory* factory = new opl::TransportModules::StateFactory();
    return factory->createState(objects, predicateMapping, functionMapping, predicateConstants, numericConstants);
}

namespace opl
{

namespace TransportModules
{

StateFactory::StateFactory()
{
}

StateFactory::~StateFactory()
{
}

opl::interface::AbstractState* StateFactory::instantiateState(const ObjectStringList& objects)
{
    State* state = new State();

    // create objects
    for (ObjectStringList::const_iterator objectIterator = objects.begin(); objectIterator != objects.end(); objectIterator++)
    {
        createObject(state, *objectIterator);
    }

    return state;
}

void StateFactory::createObject(State* state, const std::pair<std::string, std::string>& objectString)
{
    if (objectString.first == "target")
    {
        Target* object = new Target(objectString.second);
        state->targets.insert(make_pair(objectString.second, object));
        state->objects.insert(make_pair(objectString.second, object));
    }
    else if (objectString.first == "location")
    {
        Location* object = new Location(objectString.second);
        state->locations.insert(make_pair(objectString.second, object));
        state->objects.insert(make_pair(objectString.second, object));
    }
    else if (objectString.first == "locatable")
    {
        Locatable* object = new Locatable(objectString.second);
        state->locatables.insert(make_pair(objectString.second, object));
        state->objects.insert(make_pair(objectString.second, object));
    }
    else if (objectString.first == "vehicle")
    {
        Vehicle* object = new Vehicle(objectString.second);
        state->vehicles.insert(make_pair(objectString.second, object));
        state->locatables.insert(make_pair(objectString.second, object));
        state->objects.insert(make_pair(objectString.second, object));
    }
    else if (objectString.first == "package")
    {
        Package* object = new Package(objectString.second);
        state->packages.insert(make_pair(objectString.second, object));
        state->locatables.insert(make_pair(objectString.second, object));
        state->objects.insert(make_pair(objectString.second, object));
    }
    else  if (objectString.first == "object")
    {
        std::cout << "ERROR: can not allocate abstact type: object" << std::endl;
        assert(0);
    }
    else
    {
        std::cout << "Unknown object type: " << objectString.first << std::endl;
        assert(0);
    }
}

}

}
