#ifndef SEARCH_ENGINE_H
#define SEARCH_ENGINE_H

#include <vector>
#include <time.h>
#include "globals.h"

class Operator;
class TimeStampedState;

class SearchEngine
{
    private:
        bool solved;
        bool solved_at_least_once;
        Plan plan;
        PlanTrace path;
    public:
        enum status
        {
            FAILED, SOLVED, IN_PROGRESS
        };
    protected:
        virtual enum status step() = 0;

        void set_plan(const Plan &plan);
        void set_path(const PlanTrace &states);
    public:
        SearchEngine();
        virtual ~SearchEngine();
        virtual void statistics(time_t & current_time) const;
        virtual void initialize()
        {
        }
        virtual void dump_everything() const = 0;
        bool found_solution() const;
        bool found_at_least_one_solution() const;
        const Plan &get_plan() const;
        const PlanTrace& get_path() const;
        enum status search();
};

#endif
