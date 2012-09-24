#include "planner_modules_pr2/module_param_cache.h"

std::string createPoseParamString(const geometry_msgs::Pose & pose, double precPose, double precQuat)
{
    std::stringstream ss;
    ss.precision(0);
    ss << std::fixed;
    // the actual values don't matter as long as they are unique for this pose 
    // cannot use doubles here, as param keys are not allowed to contain '.' or '-'
    ss << "x";
    if(pose.position.x < 0)
        ss << "N";
    ss << abs(static_cast<int>(1.0/precPose * pose.position.x));
    ss << "y";
    if(pose.position.y < 0)
        ss << "N";
    ss << abs(static_cast<int>(1.0/precPose * pose.position.y));
    ss << "z";
    if(pose.position.z < 0)
        ss << "N";
    ss << abs(static_cast<int>(1.0/precPose * pose.position.z));

    ss << "qx";
    if(pose.orientation.x < 0)
        ss << "N";
    ss << abs(static_cast<int>(1.0/precQuat * pose.orientation.x));
    ss << "qy";
    if(pose.orientation.y < 0)
        ss << "N";
    ss << abs(static_cast<int>(1.0/precQuat * pose.orientation.y));
    ss << "qz";
    if(pose.orientation.z < 0)
        ss << "N";
    ss << abs(static_cast<int>(1.0/precQuat * pose.orientation.z));
    ss << "qw";
    if(pose.orientation.w < 0)
        ss << "N";
    ss << abs(static_cast<int>(1.0/precQuat * pose.orientation.w));

    return ss.str();
}

