#include "tidyup_recognition_actions/actionExecutorObjectRecognition.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_recognition_actions, action_executor_object_recognition,
		tidyup_recognition_actions::ActionExecutorObjectRecognition,
        continual_planning_executive::ActionExecutorInterface)


namespace tidyup_recognition_actions
{

    bool ActionExecutorObjectRecognition::fillGoal(tidyup_msgs::RecognizeObjectGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
//        goal.target_pose.header.frame_id = "/map";      // TODO: whole thing tf frame resolv?
//        goal.target_pose.header.stamp = ros::Time::now();
//
//        ROS_ASSERT(a.parameters.size() == 2);
//        string targetName = a.parameters[1];
//
//        // extract nicer + warn.
//        Predicate p;
//        p.parameters.push_back(targetName);
//        p.name = "x";
//        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.position.x))
//            return false;
//        p.name = "y";
//        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.position.y))
//            return false;
//        p.name = "z";
//        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.position.z))
//            return false;
//        p.name = "qx";
//        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.orientation.x))
//            return false;
//        p.name = "qy";
//        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.orientation.y))
//            return false;
//        p.name = "qz";
//        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.orientation.z))
//            return false;
//        p.name = "qw";
//        if(!current.hasNumericalFluent(p, &goal.target_pose.pose.orientation.w))
//            return false;
        return true;
    }

    void ActionExecutorObjectRecognition::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const tidyup_msgs::RecognizeObjectResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_ASSERT(a.parameters.size() == 2);
            string targetName = a.parameters[1];
            current.setBooleanPredicate("explored", targetName, true);
        }
    }

};

