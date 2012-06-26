#ifndef ACTION_EXECUTOR_ARM_TO_CARRY_H
#define ACTION_EXECUTOR_ARM_TO_CARRY_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <tidyup_msgs/PostGraspPositionAction.h>

namespace tidyup_actions
{

    class ActionExecutorArmToCarry : public ActionExecutorActionlib<tidyup_msgs::PostGraspPositionAction,
        tidyup_msgs::PostGraspPositionGoal, tidyup_msgs::PostGraspPositionResult>
    {
        public:
            /**
             *  Initialize the ArmToSide action using the following parameters:
             *  action_plan_name action_server_name [armStatePredicate [armAtSideConstant]]
             *  
             *  armStatePredicate and armAtSideConstant give the predicate that is set to the at side
             *  constant when the action succeeds.
             */
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillGoal(tidyup_msgs::PostGraspPositionGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState, const tidyup_msgs::PostGraspPositionResult & result,
                    const DurativeAction & a, SymbolicState & current);

        private:
            std::string _armStatePredicateName;     // the arm state predicate name
            std::string _armAtCarryConstantName;    // the arm at carry position constant name
    };

};

#endif

