#ifndef PLAN_H
#define PLAN_H

#include <string>
#include <vector>
using std::string;
using std::vector;
#include <iostream>
#include "continual_planning_msgs/TemporalAction.h"

class DurativeAction
{
    public:
        DurativeAction() { }
        DurativeAction(const continual_planning_msgs::TemporalAction & msg);

        string name;
        vector<string> parameters;
        double duration;
        double startTime;

        /// for sorting/inserting in sets
        bool operator<(const DurativeAction & a) const;
        bool operator==(const DurativeAction & a) const;
};

std::ostream & operator<<(std::ostream & os, const DurativeAction & a);

class Plan
{
    public:
        Plan();
        ~Plan();

        bool empty() const { return actions.empty(); }

        void removeAction(const DurativeAction & a);

        vector<DurativeAction> actions;
};

std::ostream & operator<<(std::ostream & os, const Plan & p);

#endif

