#ifndef STATE_CREATOR_H
#define STATE_CREATOR_H

#include "symbolicState.h"
#include <stdio.h>
#include <string>
using std::string;

/// Base interface for creating/estimating SymbolicStates from the world.
/**
 * StateCreators are specific to a certain domain.
 * There can be multiple StateCreators that fill different aspects of the symbolic state.
 * Those should not contradict each other.
 */
class StateCreator
{
    public:
        StateCreator();
        ~StateCreator();

        virtual bool fillState(SymbolicState & state) = 0;
};

inline string makeNamedId(string name, int id)
{
    char buf[name.length() + 20];
    snprintf(buf, name.length() + 19, "%s%d", name.c_str(), id);
    return string(buf);
}

#endif

