
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
    opl::interface::AbstractStateFactory* factory = new opl::RoomScanning::StateFactory();
    return factory->createState(objects, predicateMapping, functionMapping, predicateConstants, numericConstants);
}

namespace opl
{

namespace RoomScanning
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
    if (objectString.first == "pose")
    {
        Pose* object = new Pose(objectString.second);
        state->poses.insert(make_pair(objectString.second, object));
        state->objects.insert(make_pair(objectString.second, object));
    }
    else if (objectString.first == "target")
    {
        Target* object = new Target(objectString.second);
        state->targets.insert(make_pair(objectString.second, object));
        state->poses.insert(make_pair(objectString.second, object));
        state->objects.insert(make_pair(objectString.second, object));
    }
    else if (objectString.first == "door")
    {
        Door* object = new Door(objectString.second);
        state->doors.insert(make_pair(objectString.second, object));
        state->objects.insert(make_pair(objectString.second, object));
    }
    else if (objectString.first == "robot")
    {
        Robot* object = new Robot(objectString.second);
        state->robots.insert(make_pair(objectString.second, object));
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
