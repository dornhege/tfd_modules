/*
 * OplCallbackInterface.h
 *
 *  Created on: Jul 22, 2011
 *      Author: Andreas Hertle
 */

#ifndef OPLCALLBACKINTERFACE_H_
#define OPLCALLBACKINTERFACE_H_

#include "pddlModuleTypes.h"

class TimeStampedState;

namespace opl
{

namespace interface
{

class OplCallbackInterface
{
public:
    virtual ~OplCallbackInterface() {}
    virtual void setCurrentState(const TimeStampedState* currentState) = 0;

    virtual void dump() const = 0;
};

}

}

namespace modules
{

/// Function pointer to call for a opl callback module
typedef opl::interface::OplCallbackInterface* (*oplCallbackInitType)(const ObjectTypeMap& objects,
        const PredicateMapping& predicateMapping,
        const FunctionMapping& functionMapping,
        const PredicateList& predicateConstants,
        const NumericalFluentList& numericConstants);

}

#endif /* OPLCALLBACKINTERFACE_H_ */
