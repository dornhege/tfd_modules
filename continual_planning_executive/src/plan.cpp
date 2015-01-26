#include "continual_planning_executive/plan.h"
#include <math.h>

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

DurativeAction::DurativeAction(const continual_planning_msgs::TemporalAction & msg)
{
    this->startTime = msg.start_time;
    this->duration = msg.duration;
    this->name = msg.name;
    this->parameters = msg.parameters;
}

void Plan::removeAction(const DurativeAction & a)
{
    double earliestTime = HUGE_VAL;
    vector<DurativeAction>::iterator it = actions.begin();
    while(it != actions.end()) {
        if(*it == a) {
            it = actions.erase(it);
        } else {
            if(it->startTime < earliestTime)
                earliestTime = it->startTime;
            it++;
        }
    }

    // shift all actions by the earliest time in the plan to make the first action to be at 0.0 again
    for(it = actions.begin(); it != actions.end(); it++) {
        it->startTime -= earliestTime;
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

