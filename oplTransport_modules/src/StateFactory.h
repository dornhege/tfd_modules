
#ifndef TransportModules_StateFactory_H_
#define TransportModules_StateFactory_H_

#include "tfd_modules/opl/AbstractStateFactory.h"
#include "State.h"

#ifdef __cplusplus
extern "C" {
#endif

opl::interface::OplCallbackInterface* initCallbackInterface(
        const ObjectStringList& objects,
        const PredicateMapping& predicateMapping,
        const FunctionMapping& functionMapping,
        const modules::PredicateList& predicateConstants,
        const modules::NumericalFluentList& numericConstants);

#ifdef __cplusplus
}
#endif

namespace opl
{

namespace TransportModules
{

class StateFactory : public opl::interface::AbstractStateFactory
{
public:
    StateFactory();
    virtual ~StateFactory();

    opl::interface::AbstractState* instantiateState(const ObjectStringList& objects);

private:
    void createObject(State* state, const std::pair<std::string, std::string>& objectString);
};

}

}

#endif
