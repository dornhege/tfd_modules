#ifndef PLANNER_INTERFACE_H
#define PLANNER_INTERFACE_H

#include <string>
#include "plan.h"
#include "symbolicState.h"

/// Interfacing class for calling a symbolic planner.
class PlannerInterface
{
    public:
        /// Construct an interface for the planner.
        /**
         * The given domain name should match that in the domainFile.
         */
        PlannerInterface(const std::string & domainFile);
        ~PlannerInterface();

        enum PlannerResult {
            PR_SUCCESS,             ///< A (optimal) plan was returned
            PR_SUCCESS_TIMEOUT,     ///< A plan was returned, but the planner could have searched more
            PR_FAILURE_TIMEOUT,     ///< No plan computed due to timeout
            PR_FAILURE_UNREACHABLE, ///< No plan found
            PR_FAILURE_OTHER,       ///< Undetermined failure
        };
        static std::string PlannerResultStr(enum PlannerResult pr);

        virtual PlannerResult plan(const SymbolicState & init, const SymbolicState & goal, Plan & plan) = 0;

        void setTimeout(double secs) { _timeout = secs; }

    protected:
        std::string _domainFile;

        double _timeout;
};

#endif

