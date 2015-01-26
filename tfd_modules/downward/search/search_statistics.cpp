#include "search_statistics.h"
#include "globals.h"
#include <stdio.h>

SearchStatistics::SearchStatistics()
{
    generated_states = 0;
    generated_live_branched_states = 0;

    lastDumpClosedListSize = 0;
    lastDumpGeneratedStates = 0;
    lastDumpGeneratedLiveBranchedStates = 0;
    lastDumpTime = time(NULL);
    startTime = lastDumpTime;
}

SearchStatistics::~SearchStatistics()
{
}

void SearchStatistics::countChild(int openListIndex)
{
    generated_states++;
    childrenPerOpenList[openListIndex]++;
}

void SearchStatistics::countLiveBranch(int openListIndex)
{
    generated_live_branched_states++;
    childrenPerOpenList[openListIndex]++;
}

void SearchStatistics::finishExpansion()
{
    int numChildren = 0;
    for(std::map<int, int>::iterator it = childrenPerOpenList.begin(); it != childrenPerOpenList.end(); it++) {
        std::map<int, Statistics<double> >::iterator statIt = branchingFactors.find(it->first);
        if(statIt == branchingFactors.end()) {
            char* buf = new char[1024];
            sprintf(buf, "Open List %d", it->first);
            branchingFactors[it->first] = Statistics<double>(buf);
        }
        branchingFactors[it->first].addMeasurement(it->second); // count
        numChildren += it->second;
        it->second = 0; // reset for next expansion
    }
    overallBranchingFactor.addMeasurement(numChildren);
}

void SearchStatistics::dump(unsigned int closedListSize, time_t & current_time)
{
    double dt = current_time - lastDumpTime;
    double dTotal = current_time - startTime;
    if(dt == 0)
        dt = -1.0;
    if(dTotal == 0)
        dTotal = -1.0;

    cout << "Expanded Nodes: " << closedListSize << " state(s)." << endl;
    double dClosedList = closedListSize - lastDumpClosedListSize;
    printf("Rate: %.1f Nodes/s (over %.1fs) %.1f Nodes/s (total average)\n", dClosedList/dt,
            (dt > 0 ? dt : 0), double(closedListSize)/dTotal);

    cout << "Generated Nodes: " << generated_states << " state(s)." << endl;
    double dGeneratedNodes = generated_states - lastDumpGeneratedStates;
    printf("Rate: %.1f Nodes/s (over %.1fs) %.1f Nodes/s (total average)\n", dGeneratedNodes/dt,
            (dt > 0 ? dt : 0), double(generated_states)/dTotal);

    cout << "Generated Nodes due to live branching: " << generated_live_branched_states << " state(s)." << endl;
    double dGenerated_live_branchedNodes = generated_live_branched_states - lastDumpGeneratedLiveBranchedStates;
    printf("Rate: %.1f Nodes/s (over %.1fs) %.1f Nodes/s (total average)\n",
            dGenerated_live_branchedNodes/dt, (dt > 0 ? dt : 0),
            double(generated_live_branched_states)/dTotal);

    cout << "Overall branching factor by list sizes: " << ((generated_states + generated_live_branched_states)/ (double) closedListSize) << endl;
    printf("Averaged overall branching factor: ");
    overallBranchingFactor.print();
    printf("Branching factors by open list:\n");
    for(std::map<int, Statistics<double> >::iterator it = branchingFactors.begin();
            it != branchingFactors.end(); it++) {
        it->second.print();
    }

    lastDumpGeneratedStates = generated_states;
    lastDumpGeneratedLiveBranchedStates = generated_live_branched_states;
    lastDumpClosedListSize = closedListSize;
    lastDumpTime = current_time;
}

