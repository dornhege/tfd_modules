#include "search_statistics.h"
#include "globals.h"

SearchStatistics::SearchStatistics()
{
    generated_states = 0;
    childsWithDifferentG = 0;
    childsWithSameG = 0;
    childsWithDifferentF = 0;
    childsWithSameF = 0;
    parentsWithTwoOrMoreChildsWithSameG = 0;
    parentsWithAtMostOneChildWithSameG = 0;
    parentsWithAtMostOneChild = 0;
    parentsWithTwoOrMoreChilds = 0;
        
    numberOfChildren = 0;
    numberOfChildrenWithSameG = 0;
}

SearchStatistics::~SearchStatistics()
{
}

void SearchStatistics::countChild(double parentG, double childG, double parentF, double childF)
{
    generated_states++;
    numberOfChildren++;
    if(double_equals(parentG, childG)) {
        childsWithSameG++;
        numberOfChildrenWithSameG++;
    } else {
        childsWithDifferentG++;
    }
    if(double_equals(parentF, childF)) {
        childsWithSameF++;
    } else {
        childsWithDifferentF++;
    }
}

void SearchStatistics::finishExpansion()
{
    if(numberOfChildrenWithSameG >= 2) {
        parentsWithTwoOrMoreChildsWithSameG++;
    } else {
        parentsWithAtMostOneChildWithSameG++;
    }
    if(numberOfChildren >= 2) {
        parentsWithTwoOrMoreChilds++;
    } else {
        parentsWithAtMostOneChild++;
    }

    numberOfChildrenWithSameG = 0;
    numberOfChildren = 0;
}

void SearchStatistics::dump(unsigned int closedListSize) const
{
    cout << "Generated Nodes: " << generated_states << " state(s)." << endl;
    cout << "Children with same g as parent: " << childsWithSameG << endl;
    cout << "Children with different g as parent: " << childsWithDifferentG << endl;
    cout << "Children with same f as parent: " << childsWithSameF << endl;
    cout << "Children with different f as parent: " << childsWithDifferentF << endl;
    cout << "Parents with 2 or more children: " << parentsWithTwoOrMoreChilds << endl;
    cout << "Parents with at most 1 children: " << parentsWithAtMostOneChild << endl;
    cout << "Parents with 2 or more children with same g: " << parentsWithTwoOrMoreChildsWithSameG << endl;
    cout << "Parents with at most 1 children with same g: " << parentsWithAtMostOneChildWithSameG << endl;
    cout << "Average branching factor: " << (generated_states / (double) closedListSize) << endl;
    cout << "Average branching factor on zero cost edges: " << (childsWithSameG / (double) closedListSize) << endl;
}

