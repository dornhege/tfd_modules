#include "continual_planning_executive/plan.h"

Plan::Plan()
{
}

Plan::~Plan()
{
}

bool DurativeAction::operator<(const DurativeAction & a) const
{
    if(name < a.name)
        return true;
    else if(name > a.name)
        return false;

    if(duration < a.duration)
        return true;
    else if(duration > a.duration)
        return false;

    if(startTime < a.startTime)
        return true;
    else if(startTime > a.startTime)
        return false;

    if(parameters.size() < a.parameters.size())
        return true;
    else if(parameters.size() > a.parameters.size())
        return false;

    // parameters.size == a.parameters.size
    for(unsigned int i = 0; i < parameters.size(); i++) {
        if(parameters[i] < a.parameters[i])
            return true;
        else if(parameters[i] > a.parameters[i])
            return false;
    }

    // objects are equal, i.e. NOT a < b
    return false;
}
        
bool DurativeAction::operator==(const DurativeAction & a) const
{
    if(*this < a)
        return false;
    if(a < *this)
        return false;
    return true;
}

void Plan::removeAction(const DurativeAction & a)
{
    vector<DurativeAction>::iterator it = actions.begin();
    while(it != actions.end()) {
        if(*it == a) {
            it = actions.erase(it);
        } else {
            it++;
        }
    }
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

