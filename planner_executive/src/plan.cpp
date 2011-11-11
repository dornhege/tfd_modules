#include "plan.h"

Plan::Plan()
{
}

Plan::~Plan()
{
}

std::ostream & operator<<(std::ostream & os, const DurativeAction & a)
{
    os << a.startTime << ": (" << a.name;
    for(vector<string>::const_iterator it = a.parameters.begin(); it != a.parameters.end(); it++) {
        os << " " << *it;
    }
    os << ") [" << a.duration << "]";
    return os;
}

std::ostream & operator<<(std::ostream & os, const Plan & p)
{
    for(vector<DurativeAction>::const_iterator it = p.actions.begin(); it != p.actions.end(); it++) {
        os << *it << std::endl;
    }
    return os;
}

