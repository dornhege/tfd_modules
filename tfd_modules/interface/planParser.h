#ifndef PLAN_PARSER_H
#define PLAN_PARSER_H

#include <iostream>
#include <continual_planning_executive/plan.h>

class PlanParser
{
    private:
        PlanParser();
        ~PlanParser();

    public:
        static bool parsePlan(std::istream & is, Plan & plan);
};

#endif

