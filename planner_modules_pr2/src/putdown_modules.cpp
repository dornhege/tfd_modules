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

VERIFY_CONDITIONCHECKER_DEF(canPutdown);
VERIFY_APPLYEFFECT_DEF(updatePutdownPose);

void putdown_init(int argc, char** argv)
{
    ROS_ASSERT(argc == 0);

    // get world frame
    ros::NodeHandle nhPriv("~");
    
    ROS_INFO("Initialized Putdown Module.\n");
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

    double cost = INFINITE_COST;

    return cost;
}

int updatePutdownPose(const ParameterList & parameterList, predicateCallbackType predicateCallback, 
        numericalFluentCallbackType numericalFluentCallback, std::vector<double> & writtenVars)
{

    return 0;
}

