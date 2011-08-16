#include "no_heuristic.h"

#include "globals.h"
#include "operator.h"
#include "state.h"

using namespace std;

void NoHeuristic::initialize()
{
    cout << "Initializing no heuristic..." << endl;
}

double NoHeuristic::compute_heuristic(const TimeStampedState &state)
{
    if (state.satisfies(g_goal) && state.scheduled_effects.empty())
        return 0.0;

    /*cout << "compute_heuristic for " << endl;
     state.dump();
     */
    double maxT = 0.00001;
    // hack:
    for (int i = 0; i < state.operators.size(); i++) {
        if (state.operators[i].time_increment > maxT) {
            maxT = state.operators[i].time_increment;
        }
    }

    int subgoals = 0;
    for (std::vector<pair<int, double> >::iterator it = g_goal.begin(); it
            != g_goal.end(); it++) {
        if (state.satisfies(*it)) {
            subgoals++;
        }
    }

    //    cout << "= " << maxT << endl;

    //return 0.001;

    //return g_goal.size() - subgoals;

    //return max(0.1, 1000 - maxT); //no, waht i want ist operators.size
    return maxT;
    return 1.0;
    return (state.timestamp + 1.0);

    //    return 1;
}
