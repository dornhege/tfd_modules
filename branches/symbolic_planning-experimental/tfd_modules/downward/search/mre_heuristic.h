#ifndef MRE_HEURISTIC_H_
#define MRE_HEURISTIC_H_

#include "heuristic.h"

class TimeStampedState;

class MreHeuristic: public Heuristic
{
        enum
        {
            QUITE_A_LOT = 1000000
        };
    protected:
        virtual void initialize();
        virtual double compute_heuristic(
                const TimeStampedState &TimeStampedState);
    public:
        MreHeuristic()
        {
        }
        ;
        ~MreHeuristic()
        {
        }
        ;
        virtual bool dead_ends_are_reliable()
        {
            return true;
        }
};

#endif /*MRE_HEURISTIC_H_*/
