#include "greedy_apply_heuristic.h"

#include "globals.h"
#include "operator.h"
#include "state.h"
#include "successor_generator.h"
#include "plannerParameters.h"

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
    unsigned int depth = 0;
    while(true) {
        vector<const Operator *> all_operators;
        g_successor_generator->generate_applicable_ops(current, all_operators);

        const Operator* best = NULL;
        double bestCost = HUGE_VAL;
        for(unsigned int i = 0; i < all_operators.size(); i++) {
            if(all_operators[i]->is_applicable(current, 2)) {
                double cost = all_operators[i]->get_duration(&current, 2);
                if(cost < bestCost) {
                    bestCost = cost;
                    best = all_operators[i];
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

        depth ++;
        if(g_parameters.greedy_apply_heuristic_max_depth >= 0 &&
                depth > g_parameters.greedy_apply_heuristic_max_depth) {
            break;
        }

        current = TimeStampedState(current, *best, true).let_time_pass(false, true, false);
    }

    // cost 0.0 means no operator applicable
    if(totalCost == 0.0)
        return DEAD_END;

    return totalCost;
}
