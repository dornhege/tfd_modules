
#ifndef RoomScanning_Robot_H_
#define RoomScanning_Robot_H_

#include <tfd_modules/opl/oplObject.h>

#include "Pose.h"
namespace opl
{

namespace RoomScanning
{

class Robot : public opl::interface::Object
{
private:
    const opl::interface::FluentMapping* currentPoseVariable;
    const opl::interface::FluentMapping* busyVariable;

    
public:
    Robot(const std::string& name);
    virtual ~Robot();
    virtual void initialize();

    const opl::RoomScanning::Pose* currentPose() const;
    bool busy() const;
    const opl::RoomScanning::Pose* middle(const opl::RoomScanning::Pose* p1, const opl::RoomScanning::Pose* p2) const;
};

}

}

#endif

