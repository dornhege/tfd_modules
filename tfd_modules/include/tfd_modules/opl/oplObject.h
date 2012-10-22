
#ifndef OPL_OBJECT_H_
#define OPL_OBJECT_H_

#include <string>
#include "tfd_modules/module_api/pddlModuleTypes.h"
#include "tfd_modules/opl/ObjectLookupTable.h"

namespace opl
{

namespace interface
{

class ObjectLookupTable;
class Object
{
private:
    std::string objectID;

public:
    Object(const std::string& objectID);
    virtual ~Object();

    const std::string& getObjectID() const {return objectID;}

    virtual void initialize();
};

}

}

#endif

