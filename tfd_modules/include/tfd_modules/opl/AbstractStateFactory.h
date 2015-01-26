/*
 * AbstractStateFactory.h
 *
 *  Created on: Jul 7, 2011
 *      Author: Andreas Hertle
 */

#ifndef ABSTRACTSTATEFACTORY_H_
#define ABSTRACTSTATEFACTORY_H_

#include "tfd_modules/module_api/pddlModuleTypes.h"
#include "tfd_modules/opl/AbstractState.h"

namespace opl
{

namespace interface
{

class AbstractStateFactory
{
public:
    opl::interface::AbstractState* createState(
            const modules::ObjectTypeMap& objects,
            const modules::PredicateMapping& predicateMapping,
            const modules::FunctionMapping& functionMapping,
            const modules::PredicateList& predicateConstants,
            const modules::NumericalFluentList& numericConstants);
    virtual ~AbstractStateFactory() {}

protected:
    virtual AbstractState* instantiateState(const modules::ObjectTypeMap& objects) = 0;
    /// predicate / object fluent mappings
    void createFluentMapping(AbstractState* state, const std::pair<const std::string, modules::VarVal>& mapping);
    /// numeric fluent mappings
    void createFluentMapping(AbstractState* state, const std::pair<const std::string, int>& mapping);
    /// constant predicate / object mappings
    void createFluentMapping(AbstractState* state, modules::Predicate fluent);
    /// numeric constant mappings
    void createFluentMapping(AbstractState* state, const modules::NumericalFluent& fluent);
};

}

}

#endif /* ABSTRACTSTATEFACTORY_H_ */
