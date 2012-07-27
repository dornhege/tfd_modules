#include "planner_modules_pr2/putdown_modules.h"
#include "tidyup_utils/arm_state.h"
#include "tidyup_utils/planning_scene_interface.h"
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

void putdown_init(int argc, char** argv)
{
    ROS_ASSERT(argc == 2);

    // get world frame
    ros::NodeHandle nhPriv("~");
    std::string tfPrefix = tf::getPrefixParam(nhPriv);
    g_WorldFrame = tf::resolve(tfPrefix, argv[1]);
    ROS_INFO("World frame is: %s", g_WorldFrame.c_str());

    g_NodeHandle = new ros::NodeHandle();
    string service_name = "/tidyup/request_putdown_pose";

    while(!ros::service::waitForService(service_name, ros::Duration(3.0))) {
        ROS_ERROR("Service %s not available - waiting.", service_name.c_str());
    }

    g_GetPutdownPose = g_NodeHandle->serviceClient<tidyup_msgs::GetPutdownPose>(service_name, true);
    if(!g_GetPutdownPose) {
        ROS_FATAL("Could not initialize get putdown service from %s (client name: %s)", service_name.c_str(), g_GetPutdownPose.getService().c_str());
    }

    // empty key/no key maps to no pose
    PutdownCache::key_type empty_key;
    //g_PutdownCache.insert(make_pair(empty_key, make_pair(false, geometry_msgs::PoseStamped())));
    g_PutdownCache[empty_key] = make_pair(false, geometry_msgs::PoseStamped());

    // init arm joint states
    arm_navigation_msgs::PlanningScene scene = PlanningSceneInterface::instance()->getPlanningScene();
    ArmState rightArmAtSide("right_arm", "/arm_configurations/side_tuck/position/right_arm");
    rightArmAtSide.replaceJointPositions(scene.robot_state.joint_state);
    ArmState leftArmAtSide("left_arm", "/arm_configurations/side_tuck/position/left_arm");
    leftArmAtSide.replaceJointPositions(scene.robot_state.joint_state);
    PlanningSceneInterface::instance()->setPlanningSceneDiff(scene);

    ROS_INFO("Initialized Putdown Module.\n");
}

bool getPutdownPoses(const ParameterList & parameterList, predicateCallbackType predicateCallback,
        numericalFluentCallbackType numericalFluentCallback, tidyup_msgs::GetPutdownPose & srv)
{
    if(!fillPutdownRequest(parameterList, predicateCallback, numericalFluentCallback, srv.request)) {
        return false;
    }

    if(!g_GetPutdownPose) {
        ROS_ERROR("Persistent service connection to %s failed.", g_GetPutdownPose.getService().c_str());
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
            ROS_DEBUG("ServiceCall took: %f, avg: %f (num %f).",
                    dt.toSec(), totalCallsTime.toSec()/plannerCalls, plannerCalls);
        }

        if(srv.response.error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS) {
            ROS_INFO("Got a putdown pose.");
            return true;
        }

        ROS_WARN("GetPutdownPose failed. Reason: %s (%d)",
                arm_navigation_msgs::armNavigationErrorCodeToString(srv.response.error_code).c_str(),
                srv.response.error_code.val);
        return false;
    }

    ROS_ERROR("Failed to call service %s.", g_GetPutdownPose.getService().c_str());
    return false;
}

void addPoseRequest(NumericalFluentList & nfRequest, Parameter objectId)
{
    ParameterList params;
    params.push_back(objectId);

    nfRequest.push_back(NumericalFluent("x", params));
    nfRequest.push_back(NumericalFluent("y", params));
    nfRequest.push_back(NumericalFluent("z", params));
    nfRequest.push_back(NumericalFluent("qx", params));
    nfRequest.push_back(NumericalFluent("qy", params));
    nfRequest.push_back(NumericalFluent("qz", params));
    nfRequest.push_back(NumericalFluent("qw", params));
}

void fillPoseStamped(geometry_msgs::PoseStamped & pose, const NumericalFluentList & fluents, unsigned int startIdx)
{
    pose.header.frame_id = g_WorldFrame;
    pose.pose.position.x = fluents[startIdx + 0].value;
    pose.pose.position.y = fluents[startIdx + 1].value;
    pose.pose.position.z = fluents[startIdx + 2].value;
    pose.pose.orientation.x = fluents[startIdx + 3].value;
    pose.pose.orientation.y = fluents[startIdx + 4].value;
    pose.pose.orientation.z = fluents[startIdx + 5].value;
    pose.pose.orientation.w = fluents[startIdx + 6].value;
}

bool fillObjectsOnStatic(predicateCallbackType predicateCallback, Parameter static_object,
        vector<Parameter> & objects_on_static)
{
    PredicateList* list = NULL;
    if(!predicateCallback(list)) {
        ROS_ERROR("predicateCallback failed.");
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
    ROS_ASSERT(parameterList.size() == 4);
    Parameter robot_location = parameterList[0];
    Parameter putdown_object = parameterList[1];
    Parameter static_object = parameterList[2];
    Parameter arm = parameterList[3];

    // get objects on static object from planner interface
    vector<Parameter> objects_on_static;
    if(!fillObjectsOnStatic(predicateCallback, static_object, objects_on_static)) {
        return false;
    }

    // get poses for everything from planner interface
    NumericalFluentList nfRequest;
    nfRequest.reserve(7 * (1 + objects_on_static.size()));
    addPoseRequest(nfRequest, robot_location);
    for(vector<Parameter>::iterator it = objects_on_static.begin(); it != objects_on_static.end(); it++)
    {
        addPoseRequest(nfRequest, *it);
    }

    // perform the actual callback
    NumericalFluentList* nfRequestP = &nfRequest;
    if(!numericalFluentCallback(nfRequestP)) {
        ROS_ERROR("numericalFluentCallback failed.");
        return false;
    }

    // create the putdown pose query for service
    request.static_object = static_object.value;

    // fill planning scene
    arm_navigation_msgs::PlanningScene scene = PlanningSceneInterface::instance()->getPlanningScene();
    geometry_msgs::PoseStamped robotPose;
    fillPoseStamped(robotPose, nfRequest, 0 * 7);
    scene.robot_state.multi_dof_joint_state.poses[0] = robotPose.pose;
    scene.robot_state.multi_dof_joint_state.frame_ids[0] = robotPose.header.frame_id;

    request.putdown_object = putdown_object.value;

    int index = 0;
    for(vector<Parameter>::iterator graspableObjectIterator = objects_on_static.begin(); graspableObjectIterator != objects_on_static.end(); graspableObjectIterator++)
    {
        string object_name = graspableObjectIterator->value;
        for (std::vector< ::arm_navigation_msgs::CollisionObject>::iterator collisionObjectIterator = scene.collision_objects.begin(); collisionObjectIterator != scene.collision_objects.end(); collisionObjectIterator++)
        {
            if (collisionObjectIterator->id == object_name)
            {
                geometry_msgs::PoseStamped pose;
                fillPoseStamped(pose, nfRequest, (1 + index) * 7);
                collisionObjectIterator->poses[0] = pose.pose;
            }
        }
        index++;
    }
    request.arm = arm.value;
    return PlanningSceneInterface::instance()->setPlanningSceneDiff(scene);
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
            ROS_DEBUG("Got %d putdown module calls.\n", calls);
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
            ROS_ERROR("updatePutdownPose called and cache said no pose was found.");
            return 1;
        }
        // cache found set putdown_pose
        srv.response.putdown_pose = it->second.second;
    } else if(!getPutdownPoses(parameterList, predicateCallback, numericalFluentCallback, srv)) {
        ROS_ERROR("updatePutdownPose called and getPutdownPoses failed to produce one.");
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

