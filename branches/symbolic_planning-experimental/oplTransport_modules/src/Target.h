
#ifndef TransportModules_Target_H_
#define TransportModules_Target_H_

#include "tfd_modules/opl/oplObject.h"

namespace opl
{

namespace TransportModules
{

class Target : public opl::interface::Object
{
private:

    
public:
    Target(const std::string& name);
    virtual ~Target();
    void initialize();

};

}

}

#endif

