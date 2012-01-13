
#ifndef TransportModules_Location_H_
#define TransportModules_Location_H_

#include "tfd_modules/opl/oplObject.h"

namespace opl
{

namespace TransportModules
{

class Location : public opl::interface::Object
{
private:
    const opl::interface::FluentMapping* hasPetrolStationVariable;

    
public:
    Location(const std::string& name);
    virtual ~Location();
    void initialize();

    bool hasPetrolStation() const;
};

}

}

#endif

