#include "greedy_apply_heuristic.h"

#include "globals.h"
#include "operator.h"
#include "state.h"

using namespace std;

void GreedyApplyHeuristic::initialize()
{
    cout << "Initializing greedy apply heuristic..." << endl;
}

double GreedyApplyHeuristic::compute_heuristic(const TimeStampedState &state)
{
    if(state.satisfies(g_goal) && state.scheduled_effects.empty())
        return 0.0;

    TimeStampedState current = state;

    double totalCost = 0.0;
    // greedily apply the cheapest op
    // the WILL be an infinite loop if there are cycles in the domain!
    bool first = true;
    while(true) {
        Operator* best = NULL;
        double bestCost = HUGE_VAL;
        for(unsigned int i = 0; i < g_operators.size(); i++) {
            if(g_operators[i].is_applicable(current, true)) {
                double cost = g_operators[i].get_duration(&current, true);
                if(cost < bestCost) {
                    bestCost = cost;
                    best = &g_operators[i];
                }
            }
        }
        if(best == NULL)    // could not apply any more
            break;
        if(first) {
            first = false;
            set_preferred(best);    // the cheapest one is preferred
        }
        totalCost += bestCost;
        current = TimeStampedState(current, *best, true);
    }

    // cost 0.0 means no operator applicable
    if(totalCost == 0.0)
        return DEAD_END;

    return totalCost;
}
