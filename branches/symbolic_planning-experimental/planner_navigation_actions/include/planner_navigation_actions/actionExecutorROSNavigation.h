#ifndef ACTION_EXECUTOR_R_O_S_NAVIGATION_H
#define ACTION_EXECUTOR_R_O_S_NAVIGATION_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <vector>
#include <utility>

namespace planner_navigation_actions
{

    class ActionExecutorROSNavigation : public ActionExecutorActionlib<move_base_msgs::MoveBaseAction
                                        , move_base_msgs::MoveBaseGoal, move_base_msgs::MoveBaseResult>
    {
        public:
            /**
             * Parameters:
             * action_plan_name action_server_name 
             *      [start (<predicate> true|false)+] [goal (<predicate> true|false)+]
             * Ex: drive /move_base start visited true goal exploring true 
             */
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillGoal(move_base_msgs::MoveBaseGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const move_base_msgs::MoveBaseResult & result,
                    const DurativeAction & a, SymbolicState & current);

        protected:
            ///< predicates to set/unset for the start location on successfull execution
            std::vector<std::pair<std::string, bool> > _startPredicates;

            ///< predicates to set/unset for the goal location on successfull execution
            std::vector<std::pair<std::string, bool> > _goalPredicates;

    };

};

#endif

