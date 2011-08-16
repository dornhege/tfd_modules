#include <cassert>
using namespace std;

#include "search_engine.h"

//#include "globals.h"

SearchEngine::SearchEngine()
{
    solved = false;
    solved_at_least_once = false;
}

SearchEngine::~SearchEngine()
{
}

void SearchEngine::statistics(time_t &) const
{
}

bool SearchEngine::found_solution() const
{
    return solved;
}

bool SearchEngine::found_at_least_one_solution() const
{
    return solved_at_least_once;
}

const Plan &SearchEngine::get_plan() const
{
    assert(solved);
    return plan;
}

void SearchEngine::set_plan(const Plan &p)
{
    solved = true;
    solved_at_least_once = true;
    plan = p;
}

const PlanTrace& SearchEngine::get_path() const
{
    assert(solved);
    return path;
}

void SearchEngine::set_path(const PlanTrace &states)
{
    path = states;
}

enum SearchEngine::status SearchEngine::search()
{
    status st = IN_PROGRESS;
    while (st == IN_PROGRESS) {
        st = step();
        //    	solved = false;
    }
    if (st == FAILED) {
        // FIXME: CLEANUP!!!!!!!!!!
        solved = false;
        return FAILED;
    } else {
        return SOLVED;
    }
}

