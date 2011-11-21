#ifndef CONTINUAL_PLANNING_H
#define CONTINUAL_PLANNING_H

#include <vector>
#include <ros/ros.h>
#include "continual_planning_executive/symbolicState.h"
#include "continual_planning_executive/stateCreator.h"
#include "continual_planning_executive/goalCreator.h"
#include "continual_planning_executive/plannerInterface.h"
#include "planExecutor.h"

// constantly do the following:
// estimate current state
// check that curState + curPlan will lead to goal
// if not:
//    replan
// executros
//  check/keep running actions
//  preempt if conflicting? planner always assumes no running actions?
//  should never decide: oops s.th. changed lets redo everything
//  -- make monitoring nice/safe
//
//  Plan monitoring does not work, if move_base/make_plan fails during movement
//  also: Execution failures should trigger replan somehow: i.e. drive to gap -> fail -> replan
//  probably need partial/subgoal plan then to get away and make new situation
//
// check contiplan paper


/// The continual planning loop.
class ContinualPlanning
{
    public:
        friend bool loadStateCreators(ros::NodeHandle & nh);
        friend bool loadGoalCreators(ros::NodeHandle & nh);
        friend bool loadActionExecutors(ros::NodeHandle & nh);
        friend bool loadPlanner(ros::NodeHandle & nh);

        enum ReplanningTriggerMethod {
            ReplanAlways,
            ReplanIfLogicalStateChanged,
            ReplanByMonitoring
        };

        ContinualPlanning();
        ~ContinualPlanning();

        /// Execute one loop step
        /**
         * \returns true, if the planning loop should continue
         */
        bool loop();

        /// Does _currentState match _goal?
        bool isGoalFulfilled() const;

    protected:
        /// Update _currentState from the world.
        bool estimateCurrentState();

        /// Return a plan that reaches the goal.
        /**
         * This might either be a copy of the current plan if that
         * reaches the goal or a new plan.
         *
         * \return a new plan that reaches _goal or an empty plan, if no plan could be found
         */
        Plan monitorAndReplan() const;

        /// Returns true, if replanning needs to be done as _currentPlan in _currentState doesn't reach _goal.
        bool needReplanning() const;

    protected:
        std::vector<continual_planning_executive::StateCreator*> _stateCreators;
        continual_planning_executive::PlannerInterface* _planner;
        PlanExecutor _planExecutor;

        SymbolicState _currentState;
        SymbolicState _goal;
        Plan _currentPlan;

        ReplanningTriggerMethod _replanningTrigger;
};

#endif

