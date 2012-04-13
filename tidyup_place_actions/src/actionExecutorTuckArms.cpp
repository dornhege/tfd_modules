#include "tidyup_place_actions/actionExecutorTuckArms.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tidyup_place_actions, action_executor_tuck_arms,
        tidyup_place_actions::ActionExecutorTuckArms,
        continual_planning_executive::ActionExecutorInterface)

namespace tidyup_place_actions
{

    bool ActionExecutorTuckArms::fillGoal(pr2_common_action_msgs::TuckArmsGoal & goal,
            const DurativeAction & a, const SymbolicState & current)
    {
        goal.tuck_left = false;
        goal.tuck_right = false;

        ROS_ASSERT(a.parameters.size() == 1);
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
                current.setObjectFluent("arm-position", l_arm, "untucked");
                current.setObjectFluent("arm-position", r_arm, "untucked");
            } else if(a.name == "tuck-arms") {
                ROS_ASSERT(result.tuck_left == true);
                ROS_ASSERT(result.tuck_right == true);
                valid_name = true;
                current.setObjectFluent("arm-position", l_arm, "tucked");
                current.setObjectFluent("arm-position", r_arm, "tucked");
            } else if(a.name == "tuck-left-untuck-right") {
                ROS_ASSERT(result.tuck_left == true);
                ROS_ASSERT(result.tuck_right == false);
                valid_name = true;
                current.setObjectFluent("arm-position", l_arm, "tucked");
                current.setObjectFluent("arm-position", r_arm, "untucked");
            } else if(a.name == "untuck-left-tuck-right") {
                ROS_ASSERT(result.tuck_left == false);
                ROS_ASSERT(result.tuck_right == true);
                valid_name = true;
                current.setObjectFluent("arm-position", l_arm, "untucked");
                current.setObjectFluent("arm-position", r_arm, "tucked");
            }
        }
    }

};

