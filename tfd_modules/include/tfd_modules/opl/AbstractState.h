/*
 * AbstractState.h
 *
 *  Created on: Jun 14, 2011
 *      Author: Andreas Hertle
 */

#ifndef ABSTRACTSTATE_H_
#define ABSTRACTSTATE_H_

#include <string>
#include <map>
#include "FluentMapping.h"
#include "oplObject.h"
#include "ObjectLookupTable.h"
#include "tfd_modules/module_api/OplCallbackInterface.h"

namespace opl
{

namespace interface
{

//class AbstractModule;
typedef std::map<std::string, FluentMapping*> FluentMappingsType;
typedef std::map<std::string, Object*> ObjectMappingsType;

class AbstractState : public ObjectLookupTable, public OplCallbackInterface
{
protected:
    FluentMappingsType fluentMappings;
    ObjectMappingsType returnObjectMappings;
    ObjectMappingsType objects;
    const TimeStampedState* currentState;

    AbstractState();

public:
    virtual void initialize() = 0;
    void updateFluentMappings();
    void dump() const;
    void setCurrentState(const TimeStampedState* currentState){this->currentState = currentState;}
    const TimeStampedState* getCurrentState() const {return currentState;}

    std::string createKey(const std::string& fluentName, const std::vector<std::string>& arguments) const;
    std::string createObjectKey(int index, double value) const;
    const FluentMapping* getVariable(const std::string& key) const;
    bool getPredicateValue(const FluentMapping* variable) const;
    double getNumericValue(const FluentMapping* variable) const;
    const Object* getObject(const FluentMapping* variable) const;

    friend class AbstractStateFactory;
};

}

}

#endif /* ABSTRACTSTATE_H_ */
