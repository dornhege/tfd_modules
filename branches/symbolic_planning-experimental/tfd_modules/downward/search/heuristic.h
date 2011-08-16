#ifndef HEURISTIC_H
#define HEURISTIC_H

#include <map>
#include <vector>

class Operator;
class TimeStampedState;

class Heuristic
{
        enum
        {
            NOT_INITIALIZED = -2
        };
        double heuristic;
        std::vector<const Operator *> preferred_operators;

        struct EvaluationInfo
        {
                EvaluationInfo()
                {
                    heuristic = NOT_INITIALIZED;
                }
                EvaluationInfo(double heur,
                        const std::vector<const Operator *> &pref) :
                    heuristic(heur), preferred_operators(pref)
                {
                }
                double heuristic;
                std::vector<const Operator *> preferred_operators;
        };

        bool use_cache;
        std::map<TimeStampedState, EvaluationInfo> state_cache;
    protected:
        enum
        {
            DEAD_END = -1
        };
        virtual void initialize()
        {
        }
        virtual double compute_heuristic(const TimeStampedState &state) = 0;
    public:
        double waiting_time;
        void set_preferred(const Operator *op);
        Heuristic(bool use_cache = false);
        virtual ~Heuristic();

        double evaluate(const TimeStampedState &state);
        bool is_dead_end();
        double get_heuristic();
        void get_preferred_operators(std::vector<const Operator *> &result);
        virtual bool dead_ends_are_reliable()
        {
            return true;
        }
};

#endif
