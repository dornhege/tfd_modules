
#ifndef RoomScanning_Target_H_
#define RoomScanning_Target_H_

#include <tfd_modules/opl/oplObject.h>

#include "Pose.h"
namespace opl
{

namespace RoomScanning
{

class Target : public Pose
{
private:
    const opl::interface::FluentMapping* exploredVariable;

    
public:
    Target(const std::string& name);
    virtual ~Target();
    virtual void initialize();

    bool explored() const;
};

}

}

#endif

