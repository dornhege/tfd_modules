#include "tidyup_actions/actionExecutorPutdownObject.h"
#include <pluginlib/class_list_macros.h>
#include "tidyup_msgs/GetPutdownPose.h"
#include "arm_navigation_msgs/ArmNavigationErrorCodes.h"
#include <arm_navigation_msgs/convert_messages.h>
#include "tidyup_utils/planning_scene_interface.h"

PLUGINLIB_DECLARE_CLASS(tidyup_actions, action_executor_putdown_object,
        tidyup_actions::ActionExecutorPutdownObject,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_actions
{
//    void ActionExecutorPutdownObject::initialize(const std::deque<std::string> & arguments)
//    {
//        super::initialize(arguments);
//        ROS_ASSERT_MSG(arguments.size() >= 3, "ActionExecutorPutdownObject: service name argument is missing");
//        ros::NodeHandle nh;
//        getPutdownPoseService = nh.serviceClient<tidyup_msgs::GetPutdownPose>(arguments[2]);
//        getPutdownPoseService.waitForExistence();
//        ROS_INFO("ActionExecutorPutdownObject initialized.");
//    }

    bool ActionExecutorPutdownObject::fillGoal(tidyup_msgs::PlaceObjectGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        ROS_ASSERT(a.parameters.size() == 4);
        string location = a.parameters[0];
        goal.putdown_object = a.parameters[1];
        goal.static_object = a.parameters[2];
        goal.arm = a.parameters[3];
        return true;

//        // set arm
//        goal.left_arm = false;
//        goal.right_arm = false;
//        if (arm == "left_arm")
//        {
//            goal.left_arm = true;
//        }
//        if (arm == "right_arm")
//        {
//            goal.right_arm = true;
//        }
//
//        goal.surface_name = static_object;
//
//        // reset planning scene to the state of world.
//        PlanningSceneInterface::instance()->resetPlanningScene();
//
////        if (! getPutdownPoseService.isValid())
////        {
////            getPutdownPoseService =
////        }
//        tidyup_msgs::GetPutdownPose srv;
//        srv.request.static_object = static_object;
//        srv.request.arm = arm;
//        srv.request.putdown_object = object;
//        if (! getPutdownPoseService.call(srv))
//        {
//            ROS_ERROR("find putdown pose: service call failed.");
//            return false;
//        }
//        if (srv.response.error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS)
//        {
//            ROS_INFO("Got a putdown pose.");
//            goal.position = srv.response.putdown_pose;
//        }
//        else
//        {
//            ROS_WARN("GetPutdownPose failed. Reason: %s (%d)",
//                    arm_navigation_msgs::armNavigationErrorCodeToString(srv.response.error_code).c_str(),
//                    srv.response.error_code.val);
//            return false;
//        }
//        // set the obtained putdown pose in goal
//        goal.position = srv.response.putdown_pose;
//
//        return (goal.left_arm || goal.right_arm);
    }

    void ActionExecutorPutdownObject::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const tidyup_msgs::PlaceObjectResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        ROS_INFO("PutdownObject returned result");
        ROS_ASSERT(a.parameters.size() == 4);
        string location = a.parameters[0];
        string object = a.parameters[1];
        string static_object = a.parameters[2];
        string arm = a.parameters[3];
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("PutdownObject succeeded.");
            current.setBooleanPredicate("grasped", object + " " + arm, false);
            current.setBooleanPredicate("on", object + " " + static_object, true);
            current.setBooleanPredicate("graspable-from", object + " " + location + " " + arm, true);
            current.setBooleanPredicate("searched", location, false);
            current.setNumericalFluent("x", object, result.putdown_pose.pose.position.x);
            current.setNumericalFluent("y", object, result.putdown_pose.pose.position.y);
            current.setNumericalFluent("z", object, result.putdown_pose.pose.position.z);
            current.setNumericalFluent("qx", object, result.putdown_pose.pose.orientation.x);
            current.setNumericalFluent("qy", object, result.putdown_pose.pose.orientation.y);
            current.setNumericalFluent("qz", object, result.putdown_pose.pose.orientation.z);
            current.setNumericalFluent("qw", object, result.putdown_pose.pose.orientation.w);
            current.setNumericalFluent("timestamp", object, result.putdown_pose.header.stamp.sec);
            current.setObjectFluent("frame-id", object, result.putdown_pose.header.frame_id);
        }
        current.setObjectFluent("arm-state", arm, "arm_unknown");
        current.setBooleanPredicate("recent-detected-objects", location, false);
    }

    bool ActionExecutorPutdownObject::fillPoseStamped(const string& poseName, const SymbolicState& state, geometry_msgs::PoseStamped& poseStamped)
    {
        Predicate p;
        p.parameters.push_back(poseName);
        bool no_error = true;
        p.name = "x";
        no_error &= state.hasNumericalFluent(p, &poseStamped.pose.position.x);
        p.name = "y";
        no_error &= state.hasNumericalFluent(p, &poseStamped.pose.position.y);
        p.name = "z";
        no_error &= state.hasNumericalFluent(p, &poseStamped.pose.position.z);
        p.name = "qx";
        no_error &= state.hasNumericalFluent(p, &poseStamped.pose.orientation.x);
        p.name = "qy";
        no_error &= state.hasNumericalFluent(p, &poseStamped.pose.orientation.y);
        p.name = "qz";
        no_error &= state.hasNumericalFluent(p, &poseStamped.pose.orientation.z);
        p.name = "qw";
        no_error &= state.hasNumericalFluent(p, &poseStamped.pose.orientation.w);
        poseStamped.header.frame_id = "/map";
        return no_error;
    }

};

