
#ifndef TransportModules_Locatable_H_
#define TransportModules_Locatable_H_

#include "tfd_modules/opl/oplObject.h"

#include "Location.h"
namespace opl
{

namespace TransportModules
{

class Locatable : public opl::interface::Object
{
private:

    
public:
    Locatable(const std::string& name);
    virtual ~Locatable();
    void initialize();

    bool at(const opl::TransportModules::Location* l) const;
};

}

}

#endif

