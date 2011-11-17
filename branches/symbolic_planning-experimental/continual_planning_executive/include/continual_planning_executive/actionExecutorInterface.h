#ifndef ACTION_EXECUTOR_INTERFACE_H
#define ACTION_EXECUTOR_INTERFACE_H

#include "plan.h"
#include "symbolicState.h"
#include <string>
#include <sstream>
#include <iostream>

namespace continual_planning_executive
{

    /// An object that can execute DurativeActions from a plan.
    class ActionExecutorInterface
    {
        public:
            ActionExecutorInterface() {}
            ~ActionExecutorInterface() {}

            /// An action executor should return true, if it can execute the action
            /**
             * \param [in] a the action the be exectued
             * \param [in] currentState the current state
             */
            virtual bool canExecute(const DurativeAction & a, const SymbolicState & currentState) const = 0;

            /// Execute the action and return upon execution.
            /**
             * \param [in, out] currentState the current state, possibly updated by the action.
             * \returns true, if the action was executed successfully.
             */
            virtual bool executeBlocking(const DurativeAction & a, SymbolicState & currentState) = 0;

            // TODO non blocking call with progress ala action lib for temporal/parallel execution
    };

    /// Split a named id (e.g. robot0) in name and id (robot, 0).
    inline bool splitNamedId(const string & namedId, string & name, int & id)
    {
        string::size_type foundIdx = namedId.find_first_of("0123456789");
        if(foundIdx == string::npos)
            return false;

        name = namedId.substr(0, foundIdx);

        std::stringstream ss(namedId.substr(foundIdx));
        ss >> id;
        return true;
    }

};

#endif

