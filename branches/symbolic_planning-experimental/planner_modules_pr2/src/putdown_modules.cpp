#include "planner_modules_pr2/putdown_modules.h"
#include "planner_modules_pr2/module_param_cache.h"
//#include "tidyup_utils/arm_state.h"
//#include "tidyup_utils/planning_scene_interface.h"
#include "planner_modules_pr2/tidyup_planning_scene_updater.h"
#include "tidyup_utils/planning_scene_interface.h"
//#include "tidyup_utils/transformer.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <set>
using std::set;
using std::map;
#include <utility>
using std::pair; using std::make_pair;
//#include <boost/foreach.hpp>
//#ifdef __CDT_PARSER__
//#define forEach(a, b) for(a : b)
//#else
//#define forEach BOOST_FOREACH
//#endif
#include <sys/times.h>
//#include <tf/tf.h>
//#include <tf/transform_listener.h>
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
#include <arm_navigation_msgs/convert_messages.h>
//#include <boost/tuple/tuple.hpp>
//#include <boost/tuple/tuple_comparison.hpp>
#include <tidyup_msgs/GetPutdownPose.h>
#include <sstream>
#include "tidyup_utils/stringutil.h"

VERIFY_CONDITIONCHECKER_DEF(canPutdown);
VERIFY_APPLYEFFECT_DEF(updatePutdownPose);

string g_WorldFrame;
ros::NodeHandle* g_NodeHandle = NULL;
ros::ServiceClient g_GetPutdownPose;

ModuleParamCacheString paramCache;
string separator = " ";

string logName;
geometry_msgs::Pose defaultAttachPose;


void putdown_init(int argc, char** argv)
{
    ROS_ASSERT(argc == 2);
    logName = "[putdownModule]";

    g_NodeHandle = new ros::NodeHandle();
    string service_name = "/tidyup/request_putdown_pose";

    while (!ros::service::waitForService(service_name, ros::Duration(3.0)))
    {
        ROS_ERROR("%s Service %s not available - waiting.", logName.c_str(), service_name.c_str());
    }

    g_GetPutdownPose = g_NodeHandle->serviceClient<tidyup_msgs::GetPutdownPose>(service_name, true);
    if (!g_GetPutdownPose)
    {
        ROS_FATAL("%s Could not initialize get putdown service from %s (client name: %s)", logName.c_str(), service_name.c_str(), g_GetPutdownPose.getService().c_str());
    }

    // initialize cache
    paramCache.initialize("putdown", g_NodeHandle);

    defaultAttachPose.position.x = 0.032;
    defaultAttachPose.position.y = 0.015;
    defaultAttachPose.position.z = 0.0;
    defaultAttachPose.orientation.x = 0.707;
    defaultAttachPose.orientation.y = -0.106;
    defaultAttachPose.orientation.z = -0.690;
    defaultAttachPose.orientation.w = 0.105;

    ROS_INFO("%s Initialized Putdown Module.\n", logName.c_str());
}

bool callFindPutdownPoseService(tidyup_msgs::GetPutdownPose & srv)
{
    if (!g_GetPutdownPose)
    {
        ROS_ERROR("%s Persistent service connection to %s failed.", logName.c_str(), g_GetPutdownPose.getService().c_str());
        // FIXME reconnect - this shouldn't happen.
        return false;
    }

    // statistics about using the ros service
    static double plannerCalls = 0;
    static ros::Duration totalCallsTime = ros::Duration(0.0);
    plannerCalls += 1.0;
    ros::Time callStartTime = ros::Time::now();

    // perform the actual path planner call
    if (! g_GetPutdownPose.call(srv))
    {
        ROS_ERROR("%s Failed to call service %s.", logName.c_str(), g_GetPutdownPose.getService().c_str());
        return false;
    }
    if (g_Debug)
    {
        ros::Time callEndTime = ros::Time::now();
        ros::Duration dt = callEndTime - callStartTime;
        totalCallsTime += dt;
        ROS_DEBUG("%s ServiceCall took: %f, avg: %f (num %f).", logName.c_str(),
                dt.toSec(), totalCallsTime.toSec()/plannerCalls, plannerCalls);
    }
//
//    if (srv.response.error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS)
//    {
//        ROS_INFO("%s Got a putdown pose.", logName.c_str());
//        return true;
//    }
//
//    ROS_WARN("%s GetPutdownPose failed. Reason: %s (%d)", logName.c_str(),
//            arm_navigation_msgs::armNavigationErrorCodeToString(srv.response.error_code).c_str(),
//            srv.response.error_code.val);
    return true;
}

string writePoseToString(const geometry_msgs::Pose& pose)
{
    std::stringstream stream;
    stream.precision(0);
    stream << std::fixed;
    stream << pose.position.x << separator;
    stream << pose.position.y << separator;
    stream << pose.position.z << separator;
    stream << pose.orientation.x << separator;
    stream << pose.orientation.y << separator;
    stream << pose.orientation.z << separator;
    stream << pose.orientation.w << separator;
    return stream.str();
}

bool readPoseFromString(const string cacheValue, geometry_msgs::Pose& pose)
{
    std::stringstream stream;
    stream << cacheValue;
    vector<double> coordinates;
    coordinates.resize(7);
    for(size_t i = 0; ! stream.eof() && i < coordinates.size(); i++)
    {
        stream >> coordinates[i];
    }
    if (!stream.good())
        return false;
    pose.position.x = coordinates[0];
    pose.position.y = coordinates[1];
    pose.position.z = coordinates[2];
    pose.orientation.x = coordinates[3];
    pose.orientation.y = coordinates[4];
    pose.orientation.z = coordinates[5];
    pose.orientation.w = coordinates[6];
    return true;
}

string createCacheKey(const string& putdownObject,
        const string& arm,
        const string& staticObject,
        const map<string, geometry_msgs::Pose>& movableObjects,
        const map<string, string>& objectsOnStatic)
{
    ROS_DEBUG("%s createCacheKey %s %s %s", logName.c_str(), putdownObject.c_str(), arm.c_str(), staticObject.c_str());
    std::stringstream stream;
    stream.precision(0);
    stream << std::fixed << putdownObject << arm << staticObject;
    for (map<string, string>::const_iterator objectIt = objectsOnStatic.begin(); objectIt != objectsOnStatic.end(); objectIt++)
    {
        ROS_DEBUG("%s object %s on %s", logName.c_str(), objectIt->first.c_str(), objectIt->second.c_str());
        if (objectIt->second == staticObject)
        {
            const geometry_msgs::Pose& pose = movableObjects.find(objectIt->first)->second;
            // the actual values don't matter as long as they are unique for this object
            // cannot use doubles here, as param keys are not allowed to contain '.'
            std::string poseParamString = createPoseParamString(pose);
            stream << poseParamString;
        }
    }
    return stream.str();
}

bool findPutdownPose(const ParameterList & parameterList,
        predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback,
        geometry_msgs::Pose& putdown_pose)
{
    // (canPutdown ?o - movable_object ?a - arm ?s - static_object ?g - manipulation_location)
    ROS_ASSERT(parameterList.size() == 4);
    Parameter putdown_object = parameterList[0];
    Parameter arm = parameterList[1];
    Parameter static_object = parameterList[2];
    Parameter robot_location = parameterList[3];

    // read state
    geometry_msgs::Pose robotPose;
    map<string, geometry_msgs::Pose> movableObjects;
    map<string, string> graspedObjects;
    map<string, string> objectsOnStatic;
    set<string> openDoors;
    arm_navigation_msgs::PlanningScene world = PlanningSceneInterface::instance()->getCurrentScene();
    if (! TidyupPlanningSceneUpdater::readState(robot_location.value, predicateCallback, numericalFluentCallback, robotPose, movableObjects, graspedObjects, objectsOnStatic, openDoors))
    {
        ROS_ERROR("%s read state failed.", logName.c_str());
        return false;
    }

    tidyup_msgs::GetPutdownPose srv;
    srv.request.static_object = static_object.value;
    srv.request.putdown_object = putdown_object.value;
    srv.request.arm = arm.value;
    ROS_INFO("%s putdown request: %s, %s, %s, %s", logName.c_str(), parameterList[0].value.c_str(), parameterList[1].value.c_str(), parameterList[2].value.c_str(), parameterList[3].value.c_str());

    string cacheKey = createCacheKey(srv.request.putdown_object, srv.request.arm, srv.request.static_object, movableObjects, objectsOnStatic);
    ROS_DEBUG("%s cacheKey: %s", logName.c_str(), cacheKey.c_str());
    // retrieved cached result
    string cacheValue;
    if (paramCache.get(cacheKey, cacheValue))
    {
        // cache hit. either 'impossible' or a valid pose
        if (cacheValue == "impossible")
        {
            ROS_DEBUG("%s cache hit: impossible", logName.c_str());
            return false;
        }
        else
        {
            if (! readPoseFromString(cacheValue, putdown_pose))
            {
                ROS_ERROR("%s could not read cached value. cache may be corrupt.", logName.c_str());
                return false;
            }
            ROS_DEBUG("%s cache hit: pose from cache", logName.c_str());
            return true;
        }
    }

    // no cache entry, set planning scene
    PlanningSceneInterface::instance()->resetPlanningScene();
    ROS_DEBUG("%s set planning scene", logName.c_str());
    if (! TidyupPlanningSceneUpdater::update(robotPose, movableObjects, graspedObjects, openDoors))
    {
        ROS_ERROR("%s update planning scene failed.", logName.c_str());
        return false;
    }

    // set planning scene in server call
    srv.request.planning_scene = PlanningSceneInterface::instance()->getCurrentScene();

    // call putdown service
    ROS_INFO("%s call putdown service", logName.c_str());
    if (! callFindPutdownPoseService(srv))
    {
        ROS_ERROR("%s service call failed", logName.c_str());
        return false;
    }
    if (srv.response.error_code.val == srv.response.error_code.PLANNING_FAILED)
    {
        ROS_INFO("%s service returned: impossible", logName.c_str());
        paramCache.set(cacheKey, "impossible", true);
        return false;
    }
    if  (srv.response.error_code.val == srv.response.error_code.SUCCESS)
    {
        // insert results into cache
        ROS_INFO("%s service returned: pose found, adding to cache", logName.c_str());
        putdown_pose = srv.response.putdown_pose.pose;
        paramCache.set(cacheKey, writePoseToString(putdown_pose), true);
        return true;
    }

    ROS_ERROR("%s GetPutdownPose failed. Reason: %s (%d)", logName.c_str(),
            arm_navigation_msgs::armNavigationErrorCodeToString(srv.response.error_code).c_str(),
            srv.response.error_code.val);
    return false;
}


double canPutdown(const ParameterList & parameterList,
        predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback,
        int relaxed)
{
    if (relaxed != 0)
    {
        // skip computation for heuristic
        return 0;
    }
    if (g_Debug)
    { // prevent spamming ROS_DEBUG calls unless we really want debug
        // debugging raw planner calls
        static unsigned int calls = 0;
        calls++;
        if (calls % 10000 == 0)
        {
            ROS_DEBUG("%s Got %d putdown module calls.\n", logName.c_str(), calls);
        }
    }

    geometry_msgs::Pose putdownPose;
    if (! findPutdownPose(parameterList, predicateCallback, numericalFluentCallback, putdownPose))
    {
        return INFINITE_COST;
    }
    return 0;
}

int updatePutdownPose(const ParameterList & parameterList, predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback, std::vector<double> & writtenVars)
{
    geometry_msgs::Pose putdownPose;
    if (! findPutdownPose(parameterList, predicateCallback, numericalFluentCallback, putdownPose))
    {
        return 1;
    }
    // write the pose to state:
    writtenVars[0] = putdownPose.position.x;
    writtenVars[1] = putdownPose.position.y;
    writtenVars[2] = putdownPose.position.z;
    writtenVars[3] = putdownPose.orientation.x;
    writtenVars[4] = putdownPose.orientation.y;
    writtenVars[5] = putdownPose.orientation.z;
    writtenVars[6] = putdownPose.orientation.w;
    return 0;
}

