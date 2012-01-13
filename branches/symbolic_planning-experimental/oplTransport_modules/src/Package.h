
#ifndef TransportModules_Package_H_
#define TransportModules_Package_H_

#include "tfd_modules/opl/oplObject.h"

#include "Vehicle.h"
#include "Locatable.h"
namespace opl
{

namespace TransportModules
{

class Package : public Locatable
{
private:
    const opl::interface::FluentMapping* sizeVariable;

    
public:
    Package(const std::string& name);
    virtual ~Package();
    void initialize();

    bool in(const opl::TransportModules::Vehicle* v) const;
    double size() const;
};

}

}

#endif

