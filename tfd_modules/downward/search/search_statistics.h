#ifndef SEARCH_STATISTICS_H
#define SEARCH_STATISTICS_H

class SearchStatistics
{
    public:
        SearchStatistics();
        ~SearchStatistics();

        int generated_states;
        int statePutInOpenList;
        int statesOmmitedFromOpenList;
        int childsWithSameG;
        int childsWithDifferentG;
        int childsWithSameF;
        int childsWithDifferentF;
        int parentsWithTwoOrMoreChilds;
        int parentsWithTwoOrMoreChildsWithSameG;
        int parentsWithAtMostOneChild;
        int parentsWithAtMostOneChildWithSameG;

};

#endif

