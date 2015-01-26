/*
 * ObjectLookupTable.h
 *
 *  Created on: Jul 19, 2011
 *      Author: Andreas Hertle
 */

#ifndef OBJECTLOOKUPTABLE_H_
#define OBJECTLOOKUPTABLE_H_

#include <vector>
#include <string>
#include "tfd_modules/opl/oplObject.h"
#include "tfd_modules/opl/FluentMapping.h"

namespace opl
{

namespace interface
{

class Object;
class ObjectLookupTable
{
public:
    static ObjectLookupTable* instance;
    virtual std::string createKey(const std::string& fluentName, const std::vector<std::string>& arguments) const = 0;
    virtual std::string createObjectKey(int index, double value) const = 0;
    virtual const FluentMapping* getVariable(const std::string& key) const = 0;
    virtual bool getPredicateValue(const FluentMapping* variable) const = 0;
    virtual double getNumericValue(const FluentMapping* variable) const = 0;
    virtual const Object* getObject(const FluentMapping* variable) const = 0;
    virtual ~ObjectLookupTable(){;}
};

}

}

#endif /* OBJECTLOOKUPTABLE_H_ */
