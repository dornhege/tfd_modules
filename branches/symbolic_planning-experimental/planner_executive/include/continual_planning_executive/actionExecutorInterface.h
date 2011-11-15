#ifndef ACTION_EXECUTOR_INTERFACE_H
#define ACTION_EXECUTOR_INTERFACE_H

#include "plan.h"
#include <string>
#include <sstream>
#include <iostream>

/// An object that can execute DurativeActions from a plan.
class ActionExecutorInterface
{
    public:
        ActionExecutorInterface() {}
        ~ActionExecutorInterface() {}

        virtual bool canExecute(const DurativeAction & a) const = 0;

        virtual bool executeBlocking(const DurativeAction & a) = 0;

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

#endif

