#include "putdown_modules.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
using std::map;
#include <utility>
using std::pair; using std::make_pair;
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH
#include <sys/times.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
#include <arm_navigation_msgs/convert_messages.h>

VERIFY_CONDITIONCHECKER_DEF(canPutdown);
VERIFY_APPLYEFFECT_DEF(updatePutdownPose);

string g_WorldFrame;
ros::NodeHandle* g_NodeHandle = NULL;
ros::ServiceClient g_GetPutdownPose;

// TODO caching

void putdown_init(int argc, char** argv)
{
    ROS_ASSERT(argc == 2);

    // get world frame
    ros::NodeHandle nhPriv("~");
    std::string tfPrefix = tf::getPrefixParam(nhPriv);
    g_WorldFrame = tf::resolve(tfPrefix, argv[1]);
    ROS_INFO("World frame is: %s", g_WorldFrame.c_str());

    g_NodeHandle = new ros::NodeHandle();
    string service_name = "get_putdown_pose";

    while(!ros::service::waitForService(service_name, ros::Duration(3.0))) {
        ROS_ERROR("Service %s not available - waiting.", service_name.c_str());
    }

    g_GetPutdownPose = g_NodeHandle->serviceClient<tidyup_msgs::GetPutdownPose>(service_name, true);
    if(!g_GetPutdownPose) {
        ROS_FATAL("Could not initialize get putdown service from %s (client name: %s)", service_name.c_str(), g_GetPutdownPose.getService().c_str());
    }

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
    PredicateList* list = NULL;
    if(!predicateCallback(list)) {
        ROS_ERROR("predicateCallback failed.");
        return false;
    }
    ROS_ASSERT(list != NULL);
    vector<Parameter> objects_on_static;
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

    // get poses for everything from planner interface
    NumericalFluentList nfRequest;
    nfRequest.reserve(7 * (2 + objects_on_static.size()));
    addPoseRequest(nfRequest, robot_location);
    addPoseRequest(nfRequest, putdown_object);
    for(vector<Parameter>::iterator it = objects_on_static.begin(); it != objects_on_static.end(); it++) {
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

    fillPoseStamped(request.robot_pose, nfRequest, 0 * 7);

    request.object_to_putdown.name = putdown_object.value;
    fillPoseStamped(request.object_to_putdown.pose, nfRequest, 1 * 7);

    for(unsigned int i = 0; i < objects_on_static.size(); i++) {
        tidyup_msgs::GraspableObject go;
        go.name = objects_on_static[i].value;
        fillPoseStamped(go.pose, nfRequest, (2 + i) * 7);
        request.blocking_objects.push_back(go);
    }

    request.arm = arm.value;
    return true;
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

    tidyup_msgs::GetPutdownPose srv;
    // we don't care about the content, if we can successfully get one, we canPutdown!
    if(!getPutdownPoses(parameterList, predicateCallback, numericalFluentCallback, srv)) {
        return INFINITE_COST;
    }

    return 0;
}

int updatePutdownPose(const ParameterList & parameterList, predicateCallbackType predicateCallback, 
        numericalFluentCallbackType numericalFluentCallback, std::vector<double> & writtenVars)
{
    tidyup_msgs::GetPutdownPose srv;
    if(!getPutdownPoses(parameterList, predicateCallback, numericalFluentCallback, srv)) {
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

