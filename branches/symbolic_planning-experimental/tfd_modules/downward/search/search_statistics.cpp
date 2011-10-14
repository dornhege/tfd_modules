#include "search_statistics.h"
#include "globals.h"
#include <stdio.h>

SearchStatistics::SearchStatistics()
{
    generated_states = 0;
    parentsWithAtMostOneChild = 0;
    parentsWithTwoOrMoreChilds = 0;
        
    numberOfChildren = 0;

    lastDumpClosedListSize = 0;
    lastDumpGeneratedStates = 0;
    lastDumpTime = time(NULL);
    startTime = lastDumpTime;
}

SearchStatistics::~SearchStatistics()
{
}

void SearchStatistics::countChild()
{
    generated_states++;
    numberOfChildren++;
}

void SearchStatistics::finishExpansion()
{
    if(numberOfChildren >= 2) {
        parentsWithTwoOrMoreChilds++;
    } else {
        parentsWithAtMostOneChild++;
    }

    numberOfChildren = 0;
}

void SearchStatistics::dump(unsigned int closedListSize, time_t & current_time)
{
    double dt = current_time - lastDumpTime;
    double dTotal = current_time - startTime;

    cout << "Expanded Nodes: " << closedListSize << " state(s)." << endl;
    double dClosedList = closedListSize - lastDumpClosedListSize;
    printf("Rate: %.1f Nodes/s (over %.1fs) %.1f Nodes/s (total average)\n", dClosedList/dt, dt,
            double(closedListSize)/dTotal);

    cout << "Generated Nodes: " << generated_states << " state(s)." << endl;
    double dGeneratedNodes = generated_states - lastDumpGeneratedStates;
    printf("Rate: %.1f Nodes/s (over %.1fs) %.1f Nodes/s (total average)\n", dGeneratedNodes/dt, dt,
            double(generated_states)/dTotal);

    cout << "Parents with 2 or more children: " << parentsWithTwoOrMoreChilds << endl;
    cout << "Parents with at most 1 children: " << parentsWithAtMostOneChild << endl;
    cout << "Average branching factor: " << (generated_states / (double) closedListSize) << endl;
    // TODO: also take the real branching factor in childs for each parent here (by queue/total)

    lastDumpGeneratedStates = generated_states;
    lastDumpClosedListSize = closedListSize;
    lastDumpTime = current_time;
}

