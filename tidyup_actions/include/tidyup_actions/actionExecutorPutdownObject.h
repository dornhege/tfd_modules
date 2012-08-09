#ifndef ACTION_EXECUTOR_PLACE_OBJECT_H
#define ACTION_EXECUTOR_PLACE_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <tidyup_msgs/PlaceObjectAction.h>

namespace tidyup_actions
{

class ActionExecutorPutdownObject: public ActionExecutorActionlib<tidyup_msgs::PlaceObjectAction,
        tidyup_msgs::PlaceObjectGoal, tidyup_msgs::PlaceObjectResult>
{
//private:
//    typedef ActionExecutorActionlib<tidyup_msgs::PlaceObjectAction,
//            tidyup_msgs::PlaceObjectGoal, tidyup_msgs::PlaceObjectResult> super;
public:
//    virtual void initialize(const std::deque<std::string> & arguments);
    virtual bool fillGoal(tidyup_msgs::PlaceObjectGoal & goal,
            const DurativeAction & a, const SymbolicState & current);

    virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const tidyup_msgs::PlaceObjectResult & result,
            const DurativeAction & a, SymbolicState & current);

private:
    bool fillPoseStamped(const string& poseName, const SymbolicState& state, geometry_msgs::PoseStamped& poseStamped);
//    ros::ServiceClient getPutdownPoseService;
};

}
;

#endif

