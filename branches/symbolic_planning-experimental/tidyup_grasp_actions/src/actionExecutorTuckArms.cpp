#include "tidyup_grasp_actions/actionExecutorTuckArms.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_grasp_actions, action_executor_tuck_arms,
        tidyup_grasp_actions::ActionExecutorTuckArms,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_grasp_actions
{

    bool ActionExecutorTuckArms::fillGoal(pr2_common_action_msgs::TuckArmsGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        goal.tuck_left = false;
        goal.tuck_right = false;

        ROS_ASSERT(a.parameters.size() == 2);
        string l_arm = a.parameters[0];
        string r_arm = a.parameters[1];
        ROS_ASSERT(l_arm == "left_arm");
        ROS_ASSERT(r_arm == "right_arm");

        bool valid_name = false;

        if(a.name == "untuck-arms") {
            goal.tuck_left = false;
            goal.tuck_right = false;
            valid_name = true;
        } else if(a.name == "tuck-arms") {
            goal.tuck_left = true;
            goal.tuck_right = true;
            valid_name = true;
        } else if(a.name == "tuck-left-untuck-right") {
            goal.tuck_left = true;
            goal.tuck_right = false;
            valid_name = true;
        } else if(a.name == "untuck-left-tuck-right") {
            goal.tuck_left = false;
            goal.tuck_right = true;
            valid_name = true;
        }

        return valid_name;
    }

    void ActionExecutorTuckArms::updateState(const actionlib::SimpleClientGoalState & actionReturnState,
            const pr2_common_action_msgs::TuckArmsResult & result,
            const DurativeAction & a, SymbolicState & current)
    {
        if(actionReturnState == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Tuck arms succeeded.");
            ROS_ASSERT(a.parameters.size() == 2);
            string l_arm = a.parameters[0];
            string r_arm = a.parameters[1];
            ROS_ASSERT(l_arm == "left_arm");
            ROS_ASSERT(r_arm == "right_arm");

            bool valid_name = false;

            if(a.name == "untuck-arms") {
                ROS_ASSERT(result.tuck_left == false);
                ROS_ASSERT(result.tuck_right == false);
                valid_name = true;

                current.setBooleanPredicate("untucked", l_arm, true);
                current.setBooleanPredicate("untucked", r_arm, true);
                current.setBooleanPredicate("tucked", l_arm, false);
                current.setBooleanPredicate("tucked", r_arm, false);
                current.setBooleanPredicate("post-grasped", l_arm, false);
                current.setBooleanPredicate("post-grasped", r_arm, false);
            } else if(a.name == "tuck-arms") {
                ROS_ASSERT(result.tuck_left == true);
                ROS_ASSERT(result.tuck_right == true);
                valid_name = true;

                current.setBooleanPredicate("tucked", l_arm, true);
                current.setBooleanPredicate("tucked", r_arm, true);
                current.setBooleanPredicate("untucked", l_arm, false);
                current.setBooleanPredicate("untucked", r_arm, false);
                current.setBooleanPredicate("post-grasped", l_arm, false);
                current.setBooleanPredicate("post-grasped", r_arm, false);
            } else if(a.name == "tuck-left-untuck-right") {
                ROS_ASSERT(result.tuck_left == true);
                ROS_ASSERT(result.tuck_right == false);
                valid_name = true;

                current.setBooleanPredicate("tucked", l_arm, true);
                current.setBooleanPredicate("untucked", r_arm, true);
                current.setBooleanPredicate("untucked", l_arm, false);
                current.setBooleanPredicate("tucked", r_arm, false);
                current.setBooleanPredicate("post-grasped", l_arm, false);
                current.setBooleanPredicate("post-grasped", r_arm, false);
            } else if(a.name == "untuck-left-tuck-right") {
                ROS_ASSERT(result.tuck_left == false);
                ROS_ASSERT(result.tuck_right == true);
                valid_name = true;

                current.setBooleanPredicate("untucked", l_arm, true);
                current.setBooleanPredicate("tucked", r_arm, true);
                current.setBooleanPredicate("tucked", l_arm, false);
                current.setBooleanPredicate("untucked", r_arm, false);
                current.setBooleanPredicate("post-grasped", l_arm, false);
                current.setBooleanPredicate("post-grasped", r_arm, false);
            }
        }
    }

};

