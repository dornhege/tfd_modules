
#ifndef RoomScanning_Door_H_
#define RoomScanning_Door_H_

#include <tfd_modules/opl/oplObject.h>

#include "Pose.h"
namespace opl
{

namespace RoomScanning
{

class Door : public opl::interface::Object
{
private:
    const opl::interface::FluentMapping* approachPoseVariable;
    const opl::interface::FluentMapping* openVariable;

    
public:
    Door(const std::string& name);
    virtual ~Door();
    virtual void initialize();

    const opl::RoomScanning::Pose* approachPose() const;
    bool open() const;
};

}

}

#endif

