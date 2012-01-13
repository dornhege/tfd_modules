
#ifndef TransportModules_Vehicle_H_
#define TransportModules_Vehicle_H_

#include "tfd_modules/opl/oplObject.h"

#include "Locatable.h"
namespace opl
{

namespace TransportModules
{

class Vehicle : public Locatable
{
private:
    const opl::interface::FluentMapping* readyLoadingVariable;
    const opl::interface::FluentMapping* capacityVariable;

    
public:
    Vehicle(const std::string& name);
    virtual ~Vehicle();
    void initialize();

    bool readyLoading() const;
    double capacity() const;
};

}

}

#endif

