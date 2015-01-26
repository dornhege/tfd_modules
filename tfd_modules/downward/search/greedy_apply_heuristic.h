#ifndef GREEDY_APPLY_HEURISTIC_H_
#define GREEDY_APPLY_HEURISTIC_H_

#include "heuristic.h"

class TimeStampedState;

/// Greedily apply the cheapest applicable op until no more
/// ops can be applied.
class GreedyApplyHeuristic : public Heuristic
{
        enum {
            QUITE_A_LOT = 1000000
        };
    protected:
        virtual void initialize();
        virtual double compute_heuristic(const TimeStampedState & TimeStampedState);

    public:
        GreedyApplyHeuristic() {}
        ~GreedyApplyHeuristic() {}
        virtual bool dead_ends_are_reliable() {
            return true;
        }
};

#endif

