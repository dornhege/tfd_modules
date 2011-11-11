#ifndef PLAN_H
#define PLAN_H

#include <string>
#include <vector>
using std::string;
using std::vector;
#include <iostream>

class DurativeAction
{
    public:
        string name;
        vector<string> parameters;
        double duration;
        double startTime;
};

std::ostream & operator<<(std::ostream & os, const DurativeAction & a);

class Plan
{
    public:
        Plan();
        ~Plan();

        bool empty() const { return actions.empty(); }

        vector<DurativeAction> actions;
};

std::ostream & operator<<(std::ostream & os, const Plan & p);

#endif

