#include "mre_heuristic.h"

#include "globals.h"
#include "operator.h"
#include "state.h"

using namespace std;

bool startsWith(const std::string & str, const std::string substr)
{
    return (str.find(substr) == 0);
}

void MreHeuristic::initialize()
{
    cout << "Initializing MRE heuristic..." << endl;

    assert(!g_operators.empty());

    for (vector<Operator>::iterator it = g_operators.begin(); it
            != g_operators.end(); it++) {
        if (startsWith(it->get_name(), "undock"))
            set_preferred(&(*it));
    }
}

double MreHeuristic::compute_heuristic(const TimeStampedState &state)
{
    if (state.satisfies(g_goal) && state.scheduled_effects.empty())
        return 0.0;

    double maxT = 0.00001;
    // hack:
    for (int i = 0; i < state.operators.size(); i++) {
        if (state.operators[i].time_increment > maxT) {
            maxT = state.operators[i].time_increment;
        }
    }

    // yehawww subgoals - we're in stone age of heuristics and suboptimal etc...
    int subgoals = 0;
    for (std::vector<pair<int, double> >::iterator it = g_goal.begin(); it
            != g_goal.end(); it++) {
        if (state.satisfies(*it)) {
            subgoals++;
        }
    }
    if (subgoals >= g_goal.size()) { // basically done, but possibly someone still driving somewhere so just "pull to goal"
        return 0.001;
    }

    //return 0.001;

    return g_goal.size() - subgoals;
    return max(0.1, 1000 - maxT); //no, waht i want ist operators.size
    return maxT;
    return 1.0;
    return (state.timestamp + 1.0);

    //    return 1;
}

