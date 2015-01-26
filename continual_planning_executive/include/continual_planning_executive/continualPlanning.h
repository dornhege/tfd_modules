#ifndef CONTINUAL_PLANNING_H
#define CONTINUAL_PLANNING_H

#include <vector>
#include <ros/ros.h>
#include "continual_planning_executive/symbolicState.h"
#include "continual_planning_executive/stateCreator.h"
#include "continual_planning_executive/goalCreator.h"
#include "continual_planning_executive/plannerInterface.h"
#include "continual_planning_executive/statusPublisher.h"
#include "continual_planning_executive/planExecutor.h"

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
        friend void signal_handler(int);

        enum ContinualPlanningState {
            Running,
            FinishedNoPlanToGoal,
            FinishedAtGoal
        };

        enum ReplanningTriggerMethod {
            ReplanAlways,
            ReplanIfLogicalStateChanged,
            ReplanByMonitoring
        };

        ContinualPlanning();
        ~ContinualPlanning();

        /// Execute one loop step
        /**
         * \returns Running, if the planning loop should continue
         */
        ContinualPlanningState loop();

        /// Update _currentState from the world.
        bool estimateInitialStateAndGoal();
        bool estimateCurrentState();

        /// Does _currentState match _goal?
        bool isGoalFulfilled() const;


        /// For debugging, just execute this action.
        bool executeActionDirectly(const DurativeAction & a, bool publishStatus);

        /// Force replanning in the next loop.
        void forceReplanning() { _forceReplan = true; }

        void addGoalCreator(boost::shared_ptr<continual_planning_executive::GoalCreator> gc);
        void addStateCreator(boost::shared_ptr<continual_planning_executive::StateCreator> sc);
        void addActionExecutor(boost::shared_ptr<continual_planning_executive::ActionExecutorInterface> ae);
        void setPlanner(boost::shared_ptr<continual_planning_executive::PlannerInterface> pi);

        void reset();

    protected:
        /// Return a plan that reaches the goal.
        /**
         * This might either be a copy of the current plan if that
         * reaches the goal or a new plan.
         *
         * \param [out] atGoal if the empty plan was successfully monitored, the atGoal flag is set to true
         * \return a new plan that reaches _goal or an empty plan, if no plan could be found
         */
        Plan monitorAndReplan(bool & atGoal);

        /// Returns true, if replanning needs to be done as _currentPlan in _currentState doesn't reach _goal.
        /**
         * \param [out] atGoal if the empty plan was successfully monitored, the atGoal flag is set to true
         */
        bool needReplanning(bool & atGoal) const;

    protected:
        std::vector<boost::shared_ptr<continual_planning_executive::StateCreator> > _stateCreators;
        std::vector<boost::shared_ptr<continual_planning_executive::GoalCreator> > _goalCreators;
        boost::shared_ptr<continual_planning_executive::PlannerInterface> _planner;
        PlanExecutor _planExecutor;

        SymbolicState _currentState;
        SymbolicState _goal;
        Plan _currentPlan;

        bool _forceReplan;      ///< If true, someone want to hard trigger replanning in the next step
        ReplanningTriggerMethod _replanningTrigger;

        bool _allowDirectGoalCheck;     ///< Allow to check reached goal by using the _goal instead of monitoring
        bool _initialStateEstimated;

        StatusPublisher _status;
};

#endif

