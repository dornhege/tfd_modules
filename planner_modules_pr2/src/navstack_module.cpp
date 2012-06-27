#include "navstack_module.h"
#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
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

VERIFY_CONDITIONCHECKER_DEF(pathCost);

ros::NodeHandle* g_NodeHandle = NULL;
ros::ServiceClient g_GetPlan;

/// Plan requests are issued using this frame - so the poses from the planner are given in this frame (e.g. map)
std::string g_WorldFrame;

double g_GoalTolerance = 0.5;

// Using a cache of queried path costs to prevent calling the path planning service multiple times
// Better: Can we assume symmetric path costs?
map< pair<string,string>, double> g_PathCostCache;

void navstack_init(int argc, char** argv)
{
    ROS_ASSERT(argc == 4);

    // get world frame
    ros::NodeHandle nhPriv("~");
    std::string tfPrefix = tf::getPrefixParam(nhPriv);
    g_WorldFrame = tf::resolve(tfPrefix, argv[1]);
    ROS_INFO("World frame is: %s", g_WorldFrame.c_str());

    // get goal tolerance
    char* checkPtr;
    g_GoalTolerance = strtod(argv[2], &checkPtr);
    if(checkPtr == argv[2]) {    // conversion error!
        ROS_ERROR("%s: Could not convert argument for goal tolerance: %s", __func__, argv[2]);
        g_GoalTolerance = 0.5;
    }

    if(strcmp(argv[3], "0") == 0) {
        ROS_INFO("Using absolute goal tolerance.");
    } else { // relative goal tolerance, argv[3] contains the base_local_planner namespace
        ROS_INFO("Using relative goal tolerance.");
        // get base_local_planner's xy_goal_tolerance
        std::string base_local_planner_ns = argv[3];
        double move_base_tol;
        ros::NodeHandle nh;
        if(!nh.getParam(base_local_planner_ns + "/xy_goal_tolerance", move_base_tol)) {
            ROS_ERROR_STREAM("requested relative goal tolerance, but "
                    << (base_local_planner_ns + "/xy_goal_tolerance") << " was not set"
                    << " - falling back to absolute mode");
        } else { // 2. add move_base's tolerance to our relative tolerance
            g_GoalTolerance += move_base_tol;
        }
    }

    ROS_INFO("Goal Tolerance is: %f.", g_GoalTolerance);

    // init service query for make plan
    string service_name = "move_base/make_plan";
    g_NodeHandle = new ros::NodeHandle();
    while(!ros::service::waitForService(service_name, ros::Duration(3.0))) {
        ROS_ERROR("Service %s not available - waiting.", service_name.c_str());
    }

    g_GetPlan = g_NodeHandle->serviceClient<nav_msgs::GetPlan>(service_name, true);
    if(!g_GetPlan) {
        ROS_FATAL("Could not initialize get plan service from %s (client name: %s)", service_name.c_str(), g_GetPlan.getService().c_str());
    }

    ROS_INFO("Initialized Navstack Module.\n");
}

bool fillPathRequest(const ParameterList & parameterList, numericalFluentCallbackType numericalFluentCallback,
        nav_msgs::GetPlan::Request & request)
{
    // get robot and target location from planner interface
    ROS_ASSERT(parameterList.size() == 2);

    ParameterList startParams;
    startParams.push_back(parameterList[0]);
    ParameterList goalParams;
    goalParams.push_back(parameterList[1]);
    NumericalFluentList nfRequest;
    nfRequest.reserve(14);
    nfRequest.push_back(NumericalFluent("x", startParams));
    nfRequest.push_back(NumericalFluent("y", startParams));
    nfRequest.push_back(NumericalFluent("z", startParams));
    nfRequest.push_back(NumericalFluent("qx", startParams));
    nfRequest.push_back(NumericalFluent("qy", startParams));
    nfRequest.push_back(NumericalFluent("qz", startParams));
    nfRequest.push_back(NumericalFluent("qw", startParams));
    nfRequest.push_back(NumericalFluent("x", goalParams));
    nfRequest.push_back(NumericalFluent("y", goalParams));
    nfRequest.push_back(NumericalFluent("z", goalParams));
    nfRequest.push_back(NumericalFluent("qx", goalParams));
    nfRequest.push_back(NumericalFluent("qy", goalParams));
    nfRequest.push_back(NumericalFluent("qz", goalParams));
    nfRequest.push_back(NumericalFluent("qw", goalParams));

    NumericalFluentList* nfRequestP = &nfRequest;
    if(!numericalFluentCallback(nfRequestP)) {
        ROS_ERROR("numericalFluentCallback failed.");
        return false;
    }

    // create the path planning query for service
    request.start.header.frame_id = g_WorldFrame;
    request.goal.header.frame_id = g_WorldFrame;
    request.start.pose.position.x = nfRequest[0].value;
    request.start.pose.position.y = nfRequest[1].value;
    request.start.pose.position.z = nfRequest[2].value;
    request.start.pose.orientation.x = nfRequest[3].value;
    request.start.pose.orientation.y = nfRequest[4].value;
    request.start.pose.orientation.z = nfRequest[5].value;
    request.start.pose.orientation.w = nfRequest[6].value;
    request.goal.pose.position.x = nfRequest[7].value;
    request.goal.pose.position.y = nfRequest[8].value;
    request.goal.pose.position.z = nfRequest[9].value;
    request.goal.pose.orientation.x = nfRequest[10].value;
    request.goal.pose.orientation.y = nfRequest[11].value;
    request.goal.pose.orientation.z = nfRequest[12].value;
    request.goal.pose.orientation.w = nfRequest[13].value;
    request.tolerance = g_GoalTolerance;
    return true;
}

double pathCost(const ParameterList & parameterList,
        predicateCallbackType predicateCallback, numericalFluentCallbackType numericalFluentCallback, int relaxed)
{
    if(g_Debug) {        // prevent spamming ROS_DEBUG calls unless we really want debug
        // debugging raw planner calls
        static unsigned int calls = 0;
        calls++;
        if(calls % 10000 == 0) {
            ROS_DEBUG("Got %d module calls.\n", calls);
        }
    }

    // first lookup in the cache if we answered the query already
    map<pair<string, string>, double>::iterator it = g_PathCostCache.find(make_pair(parameterList[0].value, parameterList[1].value));
    if(it != g_PathCostCache.end()) {
        return it->second;
    }

    nav_msgs::GetPlan srv;
    if(!fillPathRequest(parameterList, numericalFluentCallback, srv.request)) {
        return INFINITE_COST;
    }

    double cost = INFINITE_COST;

    if(!g_GetPlan) {
        ROS_ERROR("Persistent service connection to %s failed.", g_GetPlan.getService().c_str());
        // FIXME reconnect - this shouldn't happen.
        return INFINITE_COST;
    }

    // statistics about using the ros path planner service
    static double plannerCalls = 0;
    static ros::Duration totalCallsTime = ros::Duration(0.0);
    plannerCalls += 1.0;

    ros::Time callStartTime = ros::Time::now();
    // This construct is here, because when the robot is moving move_base will not produce other paths
    // we retry for a certain amount of time to not fail directly.
    // FIXME: Cleanup the goto code
retryGetPlan:
    static unsigned int failCounter = 0;

    // perform the actual path planner call
    if(g_GetPlan.call(srv)) {
        failCounter = 0;

        if(g_Debug) {
            ros::Time callEndTime = ros::Time::now();
            ros::Duration dt = callEndTime - callStartTime;
            totalCallsTime += dt;
            ROS_DEBUG("ServiceCall took: %f, avg: %f (num %f).", dt.toSec(), totalCallsTime.toSec()/plannerCalls, plannerCalls);
        }

        if(!srv.response.plan.poses.empty()) {
            // get plan cost
            double pathLength = 0;
            geometry_msgs::PoseStamped lastPose = srv.response.plan.poses[0];
            forEach(const geometry_msgs::PoseStamped & p, srv.response.plan.poses) {
                double d = hypot(lastPose.pose.position.x - p.pose.position.x,
                        lastPose.pose.position.y - p.pose.position.y);
                pathLength += d;
                lastPose = p;
            }
            cost = pathLength;
        } else {
            ROS_WARN("Got empty plan: %s -> %s", parameterList[0].value.c_str(), parameterList[1].value.c_str());
        }

        //ROS_INFO("Got plan: %s -> %s cost: %f.", parameterList[0].value.c_str(), parameterList[1].value.c_str(), cost);
        // also empty plan = OK or fail or none?
    } else {
        ROS_ERROR("Failed to call service %s - is the robot moving?", g_GetPlan.getService().c_str());
        failCounter++;
        if(failCounter < 300) {
            usleep(1000 * 1000);
            goto retryGetPlan;
        }
        // FIXME: what if target is unreachable, do we get false or an empty plan? i.e. is this an error
        return INFINITE_COST;
    }

    // return pathcost and cache
    g_PathCostCache[make_pair(parameterList[0].value, parameterList[1].value)] = cost;

    return cost;
}

