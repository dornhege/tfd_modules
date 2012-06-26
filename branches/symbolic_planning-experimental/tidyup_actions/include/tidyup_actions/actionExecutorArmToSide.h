#ifndef ACTION_EXECUTOR_TUCK_ARMS_H
#define ACTION_EXECUTOR_TUCK_ARMS_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <tidyup_msgs/ArmToSideAction.h>

namespace tidyup_actions
{

    class ActionExecutorArmToSide : public ActionExecutorActionlib<tidyup_msgs::ArmToSideAction,
    tidyup_msgs::ArmToSideGoal, tidyup_msgs::ArmToSideResult>
    {
        public:
            /**
             * Initialize the ArmToSide action using the following parameters:
             * action_plan_name action_server_name [armStatePredicate [armAtSideConstant]]
             *
             * armStatePredicate and armAtSideConstant give the predicate that is set to the at side
             * constant when the action succeeds.
             */
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillGoal(tidyup_msgs::ArmToSideGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState, const tidyup_msgs::ArmToSideResult & result,
                    const DurativeAction & a, SymbolicState & current);

        private:
            std::string _armStatePredicateName;     // the arm state predicate name
            std::string _armAtSideConstantName;     // the arm at side position constant name

    };

};

#endif

