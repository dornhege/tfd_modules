
#include "Pose.h"

namespace opl
{

namespace RoomScanning
{

Pose::Pose(const std::string& name)
: opl::interface::Object(name)
{
}

Pose::~Pose()
{
}

void Pose::initialize()
{
    opl::interface::Object::initialize();
    std::vector<std::string> xArguments;
    xArguments.push_back(getObjectID());

    std::string xKey = opl::interface::ObjectLookupTable::instance->createKey("Pose_x", xArguments);
    xVariable = opl::interface::ObjectLookupTable::instance->getVariable(xKey);
    std::vector<std::string> yArguments;
    yArguments.push_back(getObjectID());

    std::string yKey = opl::interface::ObjectLookupTable::instance->createKey("Pose_y", yArguments);
    yVariable = opl::interface::ObjectLookupTable::instance->getVariable(yKey);
    std::vector<std::string> zArguments;
    zArguments.push_back(getObjectID());

    std::string zKey = opl::interface::ObjectLookupTable::instance->createKey("Pose_z", zArguments);
    zVariable = opl::interface::ObjectLookupTable::instance->getVariable(zKey);
    std::vector<std::string> qxArguments;
    qxArguments.push_back(getObjectID());

    std::string qxKey = opl::interface::ObjectLookupTable::instance->createKey("Pose_qx", qxArguments);
    qxVariable = opl::interface::ObjectLookupTable::instance->getVariable(qxKey);
    std::vector<std::string> qyArguments;
    qyArguments.push_back(getObjectID());

    std::string qyKey = opl::interface::ObjectLookupTable::instance->createKey("Pose_qy", qyArguments);
    qyVariable = opl::interface::ObjectLookupTable::instance->getVariable(qyKey);
    std::vector<std::string> qzArguments;
    qzArguments.push_back(getObjectID());

    std::string qzKey = opl::interface::ObjectLookupTable::instance->createKey("Pose_qz", qzArguments);
    qzVariable = opl::interface::ObjectLookupTable::instance->getVariable(qzKey);
    std::vector<std::string> qwArguments;
    qwArguments.push_back(getObjectID());

    std::string qwKey = opl::interface::ObjectLookupTable::instance->createKey("Pose_qw", qwArguments);
    qwVariable = opl::interface::ObjectLookupTable::instance->getVariable(qwKey);

}

double Pose::x() const
{
    return opl::interface::ObjectLookupTable::instance->getNumericValue(xVariable);
}

double Pose::y() const
{
    return opl::interface::ObjectLookupTable::instance->getNumericValue(yVariable);
}

double Pose::z() const
{
    return opl::interface::ObjectLookupTable::instance->getNumericValue(zVariable);
}

double Pose::qx() const
{
    return opl::interface::ObjectLookupTable::instance->getNumericValue(qxVariable);
}

double Pose::qy() const
{
    return opl::interface::ObjectLookupTable::instance->getNumericValue(qyVariable);
}

double Pose::qz() const
{
    return opl::interface::ObjectLookupTable::instance->getNumericValue(qzVariable);
}

double Pose::qw() const
{
    return opl::interface::ObjectLookupTable::instance->getNumericValue(qwVariable);
}


}

}
