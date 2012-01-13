
#ifndef RoomScanning_Pose_H_
#define RoomScanning_Pose_H_

#include <tfd_modules/opl/oplObject.h>

namespace opl
{

namespace RoomScanning
{

class Pose : public opl::interface::Object
{
private:
    const opl::interface::FluentMapping* xVariable;
    const opl::interface::FluentMapping* yVariable;
    const opl::interface::FluentMapping* zVariable;
    const opl::interface::FluentMapping* qxVariable;
    const opl::interface::FluentMapping* qyVariable;
    const opl::interface::FluentMapping* qzVariable;
    const opl::interface::FluentMapping* qwVariable;

    
public:
    Pose(const std::string& name);
    virtual ~Pose();
    virtual void initialize();

    double x() const;
    double y() const;
    double z() const;
    double qx() const;
    double qy() const;
    double qz() const;
    double qw() const;
};

}

}

#endif

