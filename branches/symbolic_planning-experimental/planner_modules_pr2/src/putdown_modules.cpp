#include "planner_modules_pr2/putdown_modules.h"
#include "tidyup_utils/arm_state.h"
#include "tidyup_utils/planning_scene_interface.h"
#include "tidyup_utils/transformer.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <set>
using std::set;
using std::map;
#include <utility>
using std::pair; using std::make_pair;
#include <boost/foreach.hpp>
#ifdef __CDT_PARSER__
#define forEach(a, b) for(a : b)
#else
#define forEach BOOST_FOREACH
#endif
#include <sys/times.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <tidyup_msgs/GetPutdownPose.h>

VERIFY_CONDITIONCHECKER_DEF(canPutdown);
VERIFY_APPLYEFFECT_DEF(updatePutdownPose);

string g_WorldFrame;
ros::NodeHandle* g_NodeHandle = NULL;
ros::ServiceClient g_GetPutdownPose;

// all params + blocking_objs -> success + the chosen putdown pose
typedef map< boost::tuple<string,string,string,string, set<string> >,
        pair<bool, geometry_msgs::PoseStamped> > PutdownCache;

PutdownCache g_PutdownCache;
string logName;
geometry_msgs::Pose defaultAttachPose;


void putdown_init(int argc, char** argv)
{
    ROS_ASSERT(argc == 2);
    logName = "[putdownModule]";

    // get world frame
    ros::NodeHandle nhPriv("~");
    std::string tfPrefix = tf::getPrefixParam(nhPriv);
    g_WorldFrame = tf::resolve(tfPrefix, argv[1]);
    ROS_INFO_NAMED(logName, "World frame is: %s", g_WorldFrame.c_str());

    g_NodeHandle = new ros::NodeHandle();
    string service_name = "/tidyup/request_putdown_pose";

    while(!ros::service::waitForService(service_name, ros::Duration(3.0))) {
        ROS_ERROR_NAMED(logName, "Service %s not available - waiting.", service_name.c_str());
    }

    g_GetPutdownPose = g_NodeHandle->serviceClient<tidyup_msgs::GetPutdownPose>(service_name, true);
    if(!g_GetPutdownPose) {
        ROS_FATAL_NAMED(logName, "Could not initialize get putdown service from %s (client name: %s)", service_name.c_str(), g_GetPutdownPose.getService().c_str());
    }

    // empty key/no key maps to no pose
    PutdownCache::key_type empty_key;
    //g_PutdownCache.insert(make_pair(empty_key, make_pair(false, geometry_msgs::PoseStamped())));
    g_PutdownCache[empty_key] = make_pair(false, geometry_msgs::PoseStamped());

    defaultAttachPose.position.x = 0.032;
    defaultAttachPose.position.y = 0.015;
    defaultAttachPose.position.z = 0.0;
    defaultAttachPose.orientation.x = 0.707;
    defaultAttachPose.orientation.y = -0.106;
    defaultAttachPose.orientation.z = -0.690;
    defaultAttachPose.orientation.w = 0.105;

    ROS_INFO_NAMED(logName, "Initialized Putdown Module.\n");
}

bool getPutdownPoses(const ParameterList & parameterList, predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback, tidyup_msgs::GetPutdownPose & srv)
{
    if(!fillPutdownRequest(parameterList, predicateCallback, numericalFluentCallback, srv.request)) {
        return false;
    }

    if(!g_GetPutdownPose) {
        ROS_ERROR_NAMED(logName, "Persistent service connection to %s failed.", g_GetPutdownPose.getService().c_str());
        // FIXME reconnect - this shouldn't happen.
        return false;
    }

    // statistics about using the ros service
    static double plannerCalls = 0;
    static ros::Duration totalCallsTime = ros::Duration(0.0);
    plannerCalls += 1.0;
    ros::Time callStartTime = ros::Time::now();

    // perform the actual path planner call
    if(g_GetPutdownPose.call(srv)) {
        if(g_Debug) {
            ros::Time callEndTime = ros::Time::now();
            ros::Duration dt = callEndTime - callStartTime;
            totalCallsTime += dt;
            ROS_DEBUG_NAMED(logName, "ServiceCall took: %f, avg: %f (num %f).",
                    dt.toSec(), totalCallsTime.toSec()/plannerCalls, plannerCalls);
        }

        if(srv.response.error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS) {
            ROS_INFO_NAMED(logName, "Got a putdown pose.");
            return true;
        }

        ROS_WARN_NAMED(logName, "GetPutdownPose failed. Reason: %s (%d)",
                arm_navigation_msgs::armNavigationErrorCodeToString(srv.response.error_code).c_str(),
                srv.response.error_code.val);
        return false;
    }

    ROS_ERROR_NAMED(logName, "Failed to call service %s.", g_GetPutdownPose.getService().c_str());
    return false;
}

//void addPoseRequest(NumericalFluentList & nfRequest, Parameter objectId)
//{
//    ParameterList params;
//    params.push_back(objectId);
//
//    nfRequest.push_back(NumericalFluent("x", params));
//    nfRequest.push_back(NumericalFluent("y", params));
//    nfRequest.push_back(NumericalFluent("z", params));
//    nfRequest.push_back(NumericalFluent("qx", params));
//    nfRequest.push_back(NumericalFluent("qy", params));
//    nfRequest.push_back(NumericalFluent("qz", params));
//    nfRequest.push_back(NumericalFluent("qw", params));
//}

void fillPoseFromState(geometry_msgs::Pose& pose, const string& poseName, numericalFluentCallbackType numericalFluentCallback)
{
    // create the numerical fluent request
    ParameterList startParams;
    startParams.push_back(Parameter("", "", poseName));
    NumericalFluentList nfRequest;
    nfRequest.reserve(7);
    nfRequest.push_back(NumericalFluent("x", startParams));
    nfRequest.push_back(NumericalFluent("y", startParams));
    nfRequest.push_back(NumericalFluent("z", startParams));
    nfRequest.push_back(NumericalFluent("qx", startParams));
    nfRequest.push_back(NumericalFluent("qy", startParams));
    nfRequest.push_back(NumericalFluent("qz", startParams));
    nfRequest.push_back(NumericalFluent("qw", startParams));

    // get the fluents
    NumericalFluentList* nfRequestP = &nfRequest;
    if (!numericalFluentCallback(nfRequestP))
    {
        ROS_INFO("fillPoseFromState failed for object: %s", poseName.c_str());
        return;
    }

    // fill pose stamped
    pose.position.x = nfRequest[0].value;
    pose.position.y = nfRequest[1].value;
    pose.position.z = nfRequest[2].value;
    pose.orientation.x = nfRequest[3].value;
    pose.orientation.y = nfRequest[4].value;
    pose.orientation.z = nfRequest[5].value;
    pose.orientation.w = nfRequest[6].value;
}

//void fillPoseStamped(geometry_msgs::PoseStamped & pose, const NumericalFluentList & fluents, unsigned int startIdx)
//{
//    pose.header.frame_id = g_WorldFrame;
//    pose.pose.position.x = fluents[startIdx + 0].value;
//    pose.pose.position.y = fluents[startIdx + 1].value;
//    pose.pose.position.z = fluents[startIdx + 2].value;
//    pose.pose.orientation.x = fluents[startIdx + 3].value;
//    pose.pose.orientation.y = fluents[startIdx + 4].value;
//    pose.pose.orientation.z = fluents[startIdx + 5].value;
//    pose.pose.orientation.w = fluents[startIdx + 6].value;
//}

bool fillObjectsOnStatic(predicateCallbackType predicateCallback, Parameter static_object,
        vector<Parameter> & objects_on_static)
{
    PredicateList* list = NULL;
    if(!predicateCallback(list)) {
        ROS_ERROR_NAMED(logName, "predicateCallback failed.");
        return false;
    }
    ROS_ASSERT(list != NULL);
    for(PredicateList::iterator it = list->begin(); it != list->end(); it++) {
        Predicate p = *it;
        if(!p.value)
            continue;
        if(p.name != "on")
            continue;
        ROS_ASSERT(p.parameters.size() == 2);   // (on movable static)
        if(p.parameters.back().value == static_object.value) {
            objects_on_static.push_back(p.parameters.front());
        }
    }

    return true;
}

bool fillPutdownRequest(const ParameterList & parameterList, predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback, tidyup_msgs::GetPutdownPose::Request & request)
{
    // get robot location, object and static object id, and arm from parameters
    // (canPutdown ?o - movable_object ?a - arm ?s - static_object ?g - manipulation_location)
    ROS_ASSERT(parameterList.size() == 4);
    Parameter putdown_object = parameterList[0];
    Parameter arm = parameterList[1];
    Parameter static_object = parameterList[2];
    Parameter robot_location = parameterList[3];
    request.static_object = static_object.value;
    request.putdown_object = putdown_object.value;
    request.arm = arm.value;
    ROS_INFO_NAMED(logName, "putdown request: %s, %s, %s, %s", parameterList[0].value.c_str(), parameterList[1].value.c_str(), parameterList[2].value.c_str(), parameterList[3].value.c_str());

    // get objects on static object from internal state
    vector<Parameter> objects_on_static;
    if (!fillObjectsOnStatic(predicateCallback, static_object, objects_on_static))
    {
        return false;
    }

    //TODO: generalize: set all objec poses, all attached objects and correct arm states

    PlanningSceneInterface* psi = PlanningSceneInterface::instance();
    psi->resetPlanningScene();

    // set robot state in planning scene
    ROS_INFO_NAMED(logName, "update robot state in planning scene");
    arm_navigation_msgs::RobotState state = psi->getRobotState();
    fillPoseFromState(state.multi_dof_joint_state.poses[0], robot_location.value, numericalFluentCallback);
    ArmState::get("/arm_configurations/side_carry/position/", "right_arm").replaceJointPositions(state.joint_state);
    ArmState::get("/arm_configurations/side_carry/position/", "left_arm").replaceJointPositions(state.joint_state);
    psi->setRobotState(state);

    // attach putdown object to the correct arm
    ROS_INFO_NAMED(logName, "attaching object %s to arm %s", request.putdown_object.c_str(), request.arm.c_str());
//    const arm_navigation_msgs::CollisionObject* object = psi->getCollisionObject(request.putdown_object);
    psi->attachObjectToGripper(request.putdown_object, request.arm);
    psi->updateObject(request.putdown_object, defaultAttachPose);

    // update pose of graspable object in the planning scene
    for(vector<Parameter>::iterator graspableObjectIterator = objects_on_static.begin(); graspableObjectIterator != objects_on_static.end(); graspableObjectIterator++)
    {
        string object_name = graspableObjectIterator->value;
        ROS_INFO_NAMED(logName, "updating object %s", object_name.c_str());
        // if this object is attached somewhere we need to detach it
        if (psi->getAttachedCollisionObject(object_name) != NULL)
        {
            psi->detachObjectAndAdd(object_name);
        }
        // object is not attached, update pose
        if (psi->getCollisionObject(object_name) != NULL)
        {
            geometry_msgs::Pose pose;
            fillPoseFromState(pose, object_name, numericalFluentCallback);
            psi->updateObject(object_name, pose);
        }
        else
        {
            ROS_ERROR_NAMED(logName, "object %s does not exist in planning scene.", object_name.c_str());
            return false;
        }
    }
    return psi->sendDiff();
}

PutdownCache::key_type createCacheKey(const ParameterList & parameterList, predicateCallbackType predicateCallback)
{
    PutdownCache::key_type ret;

    ROS_ASSERT(parameterList.size() == 4);
    Parameter robot_location = parameterList[0];
    Parameter putdown_object = parameterList[1];
    Parameter static_object = parameterList[2];
    Parameter arm = parameterList[3];

    vector<Parameter> objects_on_static;
    if(!fillObjectsOnStatic(predicateCallback, static_object, objects_on_static)) {
        return ret;
    }
    set<string> blocking_objects;
    forEach(Parameter & p, objects_on_static) {
        blocking_objects.insert(p.value);
    }

    ret.get<0>() = robot_location.value;
    ret.get<1>() = putdown_object.value;
    ret.get<2>() = static_object.value;
    ret.get<3>() = arm.value;
    ret.get<4>() = blocking_objects;

    return ret;
}

double canPutdown(const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
    if(g_Debug) {        // prevent spamming ROS_DEBUG calls unless we really want debug
        // debugging raw planner calls
        static unsigned int calls = 0;
        calls++;
        if(calls % 10000 == 0) {
            ROS_DEBUG_NAMED(logName, "Got %d putdown module calls.\n", calls);
        }
    }

    PutdownCache::key_type key = createCacheKey(parameterList, predicateCallback);
    PutdownCache::iterator it = g_PutdownCache.find(key);
    if(it != g_PutdownCache.end()) {
        return it->second.first ? 0 : INFINITE_COST;    // mapped first should be success.
    }

    tidyup_msgs::GetPutdownPose srv;
    // we don't care about the content, if we can successfully get one, we canPutdown!
    if(!getPutdownPoses(parameterList, predicateCallback, numericalFluentCallback, srv)) {
        g_PutdownCache[key] = make_pair(false, geometry_msgs::PoseStamped());
        return INFINITE_COST;
    }

    g_PutdownCache[key] = make_pair(true, srv.response.putdown_pose);
    return 0;
}

int updatePutdownPose(const ParameterList & parameterList, predicateCallbackType predicateCallback, 
        numericalFluentCallbackType numericalFluentCallback, std::vector<double> & writtenVars)
{
    tidyup_msgs::GetPutdownPose srv;

    PutdownCache::key_type key = createCacheKey(parameterList, predicateCallback);
    PutdownCache::iterator it = g_PutdownCache.find(key);

    if(it != g_PutdownCache.end()) {
        if(!it->second.first) {
            ROS_ERROR_NAMED(logName, "updatePutdownPose called and cache said no pose was found.");
            return 1;
        }
        // cache found set putdown_pose
        srv.response.putdown_pose = it->second.second;
    } else if(!getPutdownPoses(parameterList, predicateCallback, numericalFluentCallback, srv)) {
        ROS_ERROR_NAMED(logName, "updatePutdownPose called and getPutdownPoses failed to produce one.");
        return 1;
    }

    // write the pose to state:
    writtenVars[0] = srv.response.putdown_pose.pose.position.x;
    writtenVars[1] = srv.response.putdown_pose.pose.position.y;
    writtenVars[2] = srv.response.putdown_pose.pose.position.z;
    writtenVars[3] = srv.response.putdown_pose.pose.orientation.x;
    writtenVars[4] = srv.response.putdown_pose.pose.orientation.y;
    writtenVars[5] = srv.response.putdown_pose.pose.orientation.z;
    writtenVars[6] = srv.response.putdown_pose.pose.orientation.w;

    return 0;
}

