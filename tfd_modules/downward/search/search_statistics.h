#ifndef SEARCH_STATISTICS_H
#define SEARCH_STATISTICS_H

#include <time.h>

class SearchStatistics
{
    public:
        SearchStatistics();
        ~SearchStatistics();

        int generated_states;
        int childsWithSameG;
        int childsWithDifferentG;
        int childsWithSameF;
        int childsWithDifferentF;
        int parentsWithTwoOrMoreChilds;
        int parentsWithTwoOrMoreChildsWithSameG;
        int parentsWithAtMostOneChild;
        int parentsWithAtMostOneChildWithSameG;

        /// Count statistics for one generated child
        void countChild(double parentG, double childG, double parentF, double childF);

        /// This should be called after the childs of one node have been expanded and counted to compute extra stats.
        void finishExpansion();
    
        /// Output statistics
        void dump(unsigned int closedListSize, time_t & current_time);

    private:
        // internal counters per expansion
        int numberOfChildren;
        int numberOfChildrenWithSameG;

        /// Closed list size on the last dump call
        int lastDumpClosedListSize;
        int lastDumpGeneratedStates;
        time_t lastDumpTime;
        time_t startTime;
};

#endif

