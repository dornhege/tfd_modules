#include "Robot_driveDuration_plannerCall.h"
#include "Robot_driveDuration.h"
#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
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
//#include <costmap_2d/costmap_2d.h>

/**
 * Simple module implementation for ROS navigation stack.
 *
 * Directly queries the move_base_node/make_plan service for each
 * cost request by the planner.
 */

static const bool s_Debug = false;

static ros::NodeHandle* s_NodeHandle = NULL;
static ros::ServiceClient s_GetPlan;

/// Plan requests are issued using this frame - so the poses from the planner are given in this frame (e.g. map)
static std::string s_WorldFrame;

static double s_GoalTolerance = 0.5;

//static costmap_2d::Costmap2D* s_Costmap = NULL;

// Using a cache of queried path costs to prevent calling the path planning service multiple times
// Better: Can we assume symmetric path costs?
static std::map< std::pair<std::string,std::string>, double> s_PathCostCache;

void navstack_init(int argc, char** argv)
{
   ROS_ASSERT(argc == 3);

   // get world frame
   ros::NodeHandle nhPriv("~");
   std::string tfPrefix = tf::getPrefixParam(nhPriv);
   s_WorldFrame = tf::resolve(tfPrefix, argv[1]);
   ROS_INFO("World frame is: %s", s_WorldFrame.c_str());

   // get goal tolerance
   char* checkPtr;
   s_GoalTolerance = strtod(argv[2], &checkPtr);
   if(checkPtr == argv[2]) {    // conversion error!
        ROS_ERROR("%s: Could not convert argument for goal tolerance: %s", __func__, argv[2]);
   }
   ROS_INFO("Goal Tolerance is: %f.", s_GoalTolerance);

   // init service query
   s_NodeHandle = new ros::NodeHandle();
   while(!ros::service::waitForService("move_base_node/make_plan", ros::Duration(3.0))) {
      ROS_ERROR("Service move_base_node/make_plan not available - waiting.");
   }

   s_GetPlan = s_NodeHandle->serviceClient<nav_msgs::GetPlan>("move_base_node/make_plan", true);
   if(!s_GetPlan) {
      ROS_FATAL("Could not initialize get plan service from move_base_node/make_plan (client name: %s)", s_GetPlan.getService().c_str());
   }

   ROS_INFO("Initialized Navstack Module.\n");
}

double driveDuration(
        const opl::RoomScanning::State* currentState,
        const opl::RoomScanning::Pose* origin,
        const opl::RoomScanning::Pose* destination,
        int relaxed)
{
   if(s_Debug) {        // prevent spamming ROS_DEBUG calls unless we really want debug
      // debugging raw planner calls
      static unsigned int calls = 0;
      calls ++;
      if(calls % 10000 == 0) {
         ROS_DEBUG("Got %d module calls.\n", calls);
      }
   }

   // first lookup in the cache if we answered the query already
   map<pair<string,string>, double>::iterator it = s_PathCostCache.find(make_pair(origin->getObjectID(), destination->getObjectID()));
   if(it != s_PathCostCache.end()) {
      return it->second;
   }

   // TODO: call planner directly istead
   // create the path planning query for service
   nav_msgs::GetPlan srv;
   srv.request.start.header.frame_id = s_WorldFrame;
   srv.request.goal.header.frame_id = s_WorldFrame;
   srv.request.start.pose.position.x = origin->x();
   srv.request.start.pose.position.y = origin->y();
   srv.request.start.pose.position.z = origin->z();
   srv.request.start.pose.orientation.x = origin->qx();
   srv.request.start.pose.orientation.y = origin->qy();
   srv.request.start.pose.orientation.z = origin->qz();
   srv.request.start.pose.orientation.w = origin->qw();
   srv.request.goal.pose.position.x = destination->x();
   srv.request.goal.pose.position.y = destination->y();
   srv.request.goal.pose.position.z = destination->z();
   srv.request.goal.pose.orientation.x = destination->qx();
   srv.request.goal.pose.orientation.y = destination->qy();
   srv.request.goal.pose.orientation.z = destination->qz();
   srv.request.goal.pose.orientation.w = destination->qw();
   srv.request.tolerance = s_GoalTolerance;

   double cost = INFINITE_COST;

   if(!s_GetPlan) {
      ROS_ERROR("Persistent service connection to %s failed.", s_GetPlan.getService().c_str());
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
   if(s_GetPlan.call(srv)) {
      failCounter = 0;

      if(s_Debug) {
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
      }

      ROS_INFO("Got plan from service.");
      // also empty plan = OK or fail or none?
   } else {
      ROS_ERROR("Failed to call service %s - is the robot moving?", s_GetPlan.getService().c_str());
      failCounter++;
      if(failCounter < 300) {
         usleep(1000*1000);
         goto retryGetPlan;
      }
      // FIXME: what if target is unreachable, do we get false or an empty plan? i.e. is this an error
      return INFINITE_COST;
   }

   // return pathcost and cache
   s_PathCostCache[ make_pair(origin->getObjectID(), destination->getObjectID()) ] = cost;

   return cost;
}

