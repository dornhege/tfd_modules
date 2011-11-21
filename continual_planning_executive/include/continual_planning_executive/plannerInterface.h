#ifndef PLANNER_INTERFACE_H
#define PLANNER_INTERFACE_H

#include <string>
#include <vector>
#include "continual_planning_executive/plan.h"
#include "continual_planning_executive/symbolicState.h"

namespace continual_planning_executive
{

    /// Interfacing class for calling a symbolic planner.
    class PlannerInterface
    {
        public:
            enum PlannerResult {
                PR_SUCCESS,             ///< A (optimal) plan was returned
                PR_SUCCESS_TIMEOUT,     ///< A plan was returned, but the planner could have searched more
                PR_FAILURE_TIMEOUT,     ///< No plan computed due to timeout
                PR_FAILURE_UNREACHABLE, ///< No plan found
                PR_FAILURE_OTHER,       ///< Undetermined failure
            };

            /// Construct an interface for the planner.
            PlannerInterface() {}
            virtual ~PlannerInterface() {}

            /// Initialize the planner
            /**
             * \param [in] domainFile the domain to use for planning
             * \param [in] options a list of planner specific options
             */
            virtual void initialize(const std::string & domainFile, const std::vector<std::string> & options) = 0;

            /// Execute the planner to produce a plan from init to goal.
            /**
             * \param [in] init the initial state
             * \param [in] goal the (partial) goal state
             * \param [out] plan  the created plan
             */
            virtual PlannerResult plan(const SymbolicState & init, const SymbolicState & goal, Plan & plan) = 0;
            
            /// Monitor the current plan by checking if app(init, plan) reaches goal.
            /**
             * \param [in] init the initial state
             * \param [in] goal the (partial) goal state
             * \param [in] plan  the plan to monitor
             */
            virtual PlannerResult monitor(const SymbolicState & init, const SymbolicState & goal, const Plan & plan) = 0;

            virtual void setTimeout(double secs) = 0;

            static std::string PlannerResultStr(enum PlannerResult pr);
    };

    inline std::string PlannerInterface::PlannerResultStr(enum PlannerResult pr)
    {
        switch(pr) {
            case PR_SUCCESS:
                return "PR_SUCCESS";
            case PR_SUCCESS_TIMEOUT:
                return "PR_SUCCESS_TIMEOUT";
            case PR_FAILURE_TIMEOUT:
                return "PR_FAILURE_TIMEOUT";
            case PR_FAILURE_UNREACHABLE:
                return "PR_FAILURE_UNREACHABLE";
            case PR_FAILURE_OTHER:
                return "PR_FAILURE_OTHER";
        }
        return "INVALID PLANNER RESULT";
    }

};

#endif

