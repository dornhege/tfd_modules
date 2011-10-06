#include "best_first_search.h"

#include "globals.h"
#include "heuristic.h"
#include "successor_generator.h"
#include "plannerParameters.h"
#include <time.h>
#include <iomanip>

#include <cassert>
using namespace std;

OpenListInfo::OpenListInfo(Heuristic *heur, bool only_pref)
{
    heuristic = heur;
    only_preferred_operators = only_pref;
    priority = 0;
}

BestFirstSearchEngine::BestFirstSearchEngine(QueueManagementMode _mode) :
    current_state(*g_initial_state), currentQueueIndex(-1), mode(_mode)
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
    current_predecessor = 0;
    current_operator = 0;
    start_time = time(NULL);
    bestMakespan = HUGE_VAL;
    bestSumOfGoals = HUGE_VAL;
}

BestFirstSearchEngine::~BestFirstSearchEngine()
{
}

TimeStampedState* BestFirstSearchEngine::get_current_state()
{
    return &current_state;
}


void BestFirstSearchEngine::add_heuristic(Heuristic *heuristic,
        bool use_estimates, bool use_preferred_operators)
{
    assert(use_estimates || use_preferred_operators);
    heuristics.push_back(heuristic);
    best_heuristic_values.push_back(-1);
    best_states.push_back(NULL);
    if(use_estimates) {
        open_lists.push_back(OpenListInfo(heuristic, false));
    }
    if(use_preferred_operators) {
        open_lists.push_back(OpenListInfo(heuristic, true));
        preferred_operator_heuristics.push_back(heuristic);
    }
}

void BestFirstSearchEngine::initialize()
{
    assert(!open_lists.empty());
}

void BestFirstSearchEngine::statistics(time_t & current_time) const
{
    cout << endl;
    cout << "Search Time: " << (current_time - start_time) << " sec." << endl;
    cout << "Expanded Nodes: " << closed_list.size() << " state(s)." << endl;
    cout << "Generated Nodes: " << generated_states << " state(s)." << endl;
    cout << "Children with same g as parent: " << childsWithSameG << endl;
    cout << "Children with different g as parent: " << childsWithDifferentG
        << endl;
    cout << "Children with same f as parent: " << childsWithSameF << endl;
    cout << "Children with different f as parent: " << childsWithDifferentF
        << endl;
    cout << "Parents with 2 or more children: " << parentsWithTwoOrMoreChilds
        << endl;
    cout << "Parents with at most 1 children: " << parentsWithAtMostOneChild
        << endl;
    cout << "Parents with 2 or more children with same g: "
        << parentsWithTwoOrMoreChildsWithSameG << endl;
    cout << "Parents with at most 1 children with same g: "
        << parentsWithAtMostOneChildWithSameG << endl;
    cout << "Average branching factor: " << (generated_states
            / (double) closed_list.size()) << endl;
    cout << "Average branching factor on zero cost edges: " << (childsWithSameG
            / (double) closed_list.size()) << endl;
    cout << "Best heuristic value: " << best_heuristic_values[0] << endl
        << endl;
    //    cout << "Best state:" << endl;
    //    const TimeStampedState &state = *best_states[0];
    //    if(&state) {
    //        dump_plan_prefix_for__state(state);
    //        best_states[0]->dump();
    //    }
    //    cout << endl;
}

void BestFirstSearchEngine::dump_transition() const
{
    cout << endl;
    if(current_predecessor != 0) {
        cout << "DEBUG: In step(), current predecessor is: " << endl;
        current_predecessor->dump();
    }
    cout << "DEBUG: In step(), current operator is: ";
    if(current_operator != 0) {
        current_operator->dump();
    } else {
        cout << "No operator before initial state." << endl;
    }
    cout << "DEBUG: In step(), current state is: " << endl;
    current_state.dump();
    cout << endl;
}

void BestFirstSearchEngine::dump_everything() const
{
    dump_transition();
    cout << "DEBUG: closed List is: " << endl;
    cout << endl << endl;
    for (std::vector<OpenListInfo>::const_iterator it = open_lists.begin(); it
            != open_lists.end(); it++) {
        cout << "DEBUG: an open list is:" << endl;
        // huge dangerous hack...
        OpenListEntry const* begin = &(it->open.top());
        OpenListEntry const* end = &(it->open.top()) + it->open.size();
        for (const OpenListEntry* it2 = begin; it2 != end; it2++) {
            cout << "OpenListEntry" << endl;
            cout << "state" << endl;
            std::tr1::get<0>(*it2)->dump();
            cout << "op" << endl;
            std::tr1::get<1>(*it2)->dump();
            double h = std::tr1::get<2>(*it2);
            cout << "Value: " << h << endl;
            cout << "end OpenListEntry" << endl;
        }

    }
}

SearchEngine::status BestFirstSearchEngine::step()
{
    // Invariants:
    // - current_state is the next state for which we want to compute the heuristic.
    // - current_predecessor is a permanent pointer to the predecessor of that state.
    // - current_operator is the operator which leads to current_state from predecessor.

    bool discard = true;

    double maxTimeIncrement = 0.0;
    for(int k = 0; k < current_state.operators.size(); ++k) {
        maxTimeIncrement = max(maxTimeIncrement, current_state.operators[k].time_increment);
    }
    double makeSpan = maxTimeIncrement + current_state.timestamp;

    cout << setprecision(10);
    //    cout << "makeSpan: " << makeSpan << ", bestMakespan: " << bestMakespan << endl;

    if (makeSpan < bestMakespan && !closed_list.contains(current_state)) {
        discard = false;
    }

    if(!discard) {
        // FIXME: What is the difference between parent_ptr and current_state?
        const TimeStampedState *parent_ptr = closed_list.insert(current_state,
                current_predecessor, current_operator);
        assert(&current_state != current_predecessor);

        //	cout << "................" << endl;
        //	dump_plan_prefix_for__state(*parent_ptr);
        //	parent_ptr->dump();
        //	cout << "--------" << endl;
        //	if(current_operator == g_let_time_pass) {
        //	    cout << "op: let_time_pass" << endl;
        //	} else {
        //	    cout << "op: " << current_operator->get_name() << endl;
        //	}

        //        cout << "Expanded Nodes: " << closed_list.size() << " state(s)." << endl;
        //        cout << "Prefix:" << endl;
        //    	if(current_predecessor)
        //    		dump_plan_prefix_for__state(*current_predecessor);
        //    	cout << "-" << endl;
        //    	current_state.dump();
        for(int i = 0; i < heuristics.size(); i++) {
            heuristics[i]->evaluate(current_state);
        }
        if(!is_dead_end()) {
            if(check_progress(parent_ptr)) {
                // current_state.dump();
                report_progress();
                reward_progress();
            }
            if(check_goal())
                return SOLVED;
            generate_successors(parent_ptr);
        }
    }

    //    statistics();
    //    cout << "State was: " << endl;
    //    cout << "-" << endl;
    //    dump_plan_prefix_for__state(*current_predecessor);
    //    cout << "-" << endl;
    //    current_state.dump();
    //    cout << "---------------------------------------" << endl << endl << endl;

    time_t current_time = time(NULL);
    static time_t last_stat_time = current_time;
    if(g_parameters.verbose && current_time - last_stat_time >= 10) {
        statistics(current_time);
        last_stat_time = current_time;
    }

    // use different timeouts depending if we found a plan or not.
    if(found_solution()) {
        if (g_parameters.timeout_if_plan_found > 0 
                && current_time - start_time > g_parameters.timeout_if_plan_found) {
            if(g_parameters.verbose)
                statistics(current_time);
            return SOLVED_TIMEOUT;
        }
    } else {
        if (g_parameters.timeout_while_no_plan_found > 0 
                && current_time - start_time > g_parameters.timeout_while_no_plan_found) {
            if(g_parameters.verbose)
                statistics(current_time);
            return FAILED_TIMEOUT;
        }
    }

    return fetch_next_state();
}

bool BestFirstSearchEngine::is_dead_end()
{
    // If a reliable heuristic reports a dead end, we trust it.
    // Otherwise, all heuristics must agree on dead-end-ness.
    int dead_end_counter = 0;
    for(int i = 0; i < heuristics.size(); i++) {
        if(heuristics[i]->is_dead_end()) {
            if(heuristics[i]->dead_ends_are_reliable())
                return true;
            else
                dead_end_counter++;
        }
    }
    return dead_end_counter == heuristics.size();
}

bool BestFirstSearchEngine::check_goal()
{
    // Any heuristic reports 0 iff this is a goal state, so we can
    // pick an arbitrary one.
    Heuristic *heur = open_lists[0].heuristic;
    if(!heur->is_dead_end() && heur->evaluate(current_state) == 0) {
        if(current_state.operators.size() > 0) {
            return false;
        }

        if(!current_state.satisfies(g_goal)) {  // will assert...
            dump_everything();
        }
        assert(current_state.operators.empty() && current_state.satisfies(g_goal));

        // We actually need this silly !heur->is_dead_end() check because
        // this state *might* be considered a non-dead end by the
        // overall search even though heur considers it a dead end
        // (e.g. if heur is the CG heuristic, but the FF heuristic is
        // also computed and doesn't consider this state a dead end.
        // If heur considers the state a dead end, it cannot be a goal
        // state (heur will not be *that* stupid). We may not call
        // get_heuristic() in such cases because it will barf.
        Plan plan;
        PlanTrace path;
        closed_list.trace_path(current_state, plan, path);
        set_plan(plan);
        set_path(path);
        return true;
    } else {
        return false;
    }
}

void BestFirstSearchEngine::dump_plan_prefix_for_current_state() const
{
    dump_plan_prefix_for__state(current_state);
}

void BestFirstSearchEngine::dump_plan_prefix_for__state(const TimeStampedState &state) const
{
    Plan plan;
    PlanTrace path;
    closed_list.trace_path(state, plan, path);
    for(int i = 0; i < plan.size(); i++) {
        const PlanStep& step = plan[i];
        cout << step.start_time << ": " << "(" << step.op->get_name() << ")"
            << " [" << step.duration << "]" << endl;
    }
}

bool BestFirstSearchEngine::check_progress(const TimeStampedState* state)
{
    bool progress = false;
    for(int i = 0; i < heuristics.size(); i++) {
        if(heuristics[i]->is_dead_end())
            continue;
        double h = heuristics[i]->get_heuristic();
        double &best_h = best_heuristic_values[i];
        if(best_h == -1 || h < best_h) {
            best_h = h;
            best_states[i] = state;
            progress = true;
        }
    }
    return progress;
}

void BestFirstSearchEngine::report_progress()
{
    cout << "Best heuristic value: ";
    for(int i = 0; i < heuristics.size(); i++) {
        cout << best_heuristic_values[i];
        if(i != heuristics.size() - 1)
            cout << "/";
    }
    cout << " [expanded " << closed_list.size() << " state(s)]" << endl;
}

void BestFirstSearchEngine::reward_progress()
{
    // Boost the "preferred operator" open lists somewhat whenever
    // progress is made. This used to be used in multi-heuristic mode
    // only, but it is also useful in single-heuristic mode, at least
    // in Schedule.
    //
    // Test the impact of this, and find a better way of rewarding
    // successful exploration. For example, reward only the open queue
    // from which the good state was extracted and/or the open queues
    // for the heuristic for which a new best value was found.

    for(int i = 0; i < open_lists.size(); i++)
        if(open_lists[i].only_preferred_operators)
            open_lists[i].priority -= 1000;
}

double BestFirstSearchEngine::getGc(const TimeStampedState *state)
{
    return closed_list.getCostOfPath(*state);
}

double BestFirstSearchEngine::getGc(const TimeStampedState *state,
        const Operator *op)
{
    double opCost = 0.0;
    if (op && op != g_let_time_pass) {
        opCost += op->get_duration(state);
    }
    return getGc(state) + opCost;
}

double BestFirstSearchEngine::getGm(const TimeStampedState *state)
{
    double longestActionDuration = 0.0;
    for (int i = 0; i < state->operators.size(); ++i) {
        const ScheduledOperator* op = &state->operators[i];
        double duration = 0.0;
        if (op && op != g_let_time_pass) {
            duration = op->origin->get_duration(state);  // FIXME use origin
        }
        if (duration > longestActionDuration) {
            longestActionDuration = duration;
        }
    }
    return state->timestamp + longestActionDuration;
}

double BestFirstSearchEngine::getGt(const TimeStampedState *state)
{
    return state->timestamp;
}

bool tssKnown2(ThirdClosedList& scl,
        const TimedSymbolicStates& timedSymbolicStates)
{
    assert(timedSymbolicStates.size() > 0);
    bool ret = true;
    for (int i = 0; i < timedSymbolicStates.size(); ++i) {
        if (scl.count(timedSymbolicStates[i].first) > 0) {
            double currentBestMakespan = scl[timedSymbolicStates[i].first];
            if (timedSymbolicStates[i].second + EPSILON < currentBestMakespan) {
                ret = false;
                scl[timedSymbolicStates[i].first]
                    = timedSymbolicStates[i].second;
            }
        } else {
            ret = false;
            scl[timedSymbolicStates[i].first] = timedSymbolicStates[i].second;
        }
    }
    return ret;
}

void BestFirstSearchEngine::generate_successors(const TimeStampedState *parent_ptr)
{
    vector<const Operator *> all_operators;
    g_successor_generator->generate_applicable_ops(current_state, all_operators);

    vector<const Operator *> preferred_operators;
    for(int i = 0; i < preferred_operator_heuristics.size(); i++) {
        Heuristic *heur = preferred_operator_heuristics[i];
        if(!heur->is_dead_end()) {
            heur->get_preferred_operators(preferred_operators);
        }
    }

    // HACK!?!?! Entferne pref_ops aus normaler liste
    for(int k = 0; k < preferred_operators.size(); ++k) {
        for(int l = 0; l < all_operators.size(); ++l) {
            if(all_operators[l] == preferred_operators[k]) {
                all_operators[l] = all_operators[all_operators.size() - 1];
                all_operators.pop_back();
                break;
            }
        }
    }

    //   cout << "Preferred ops: " << endl;
    //   for(int i = 0; i < preferred_operators.size(); ++i) {
    //       cout << preferred_operators[i]->get_name() << endl;
    //   }
    //   cout << "............................." << endl;
    //
    //   cout << "All ops:" << endl;
    //   for(int i = 0; i < all_operators.size(); ++i) {
    //       cout << all_operators[i]->get_name() << endl;
    //   }
    //   cout << "---------------" << endl;


    bool is_g_plateau = true;
    int i = 0;
    if (mode == HIERARCHICAL) {
        i = 1;
        double parentG = getGm(parent_ptr);
        vector<const Operator *> &ops =
            open_lists[0].only_preferred_operators ? preferred_operators
            : all_operators;
        for (int j = 0; j < ops.size(); j++) {
            assert(ops[j]->get_name().compare("wait") != 0);
            TimeStampedState tss = TimeStampedState(current_state, *ops[j]);
            double childG = getGm(&tss);
            if (!double_equals(parentG, childG)) {
                currentQueueIndex = 0;
                is_g_plateau = false;
                break;
            }
        }
        if (is_g_plateau) {
            currentQueueIndex = 1;
        }
    }

    Heuristic *heur;
    if (mode == HIERARCHICAL) {
        if (is_g_plateau) {
            heur = open_lists[1].heuristic;
        } else {
            heur = open_lists[0].heuristic;
        }
    } else {
        heur = open_lists[i].heuristic;
    }
    double parentG, parentH, parentF;

    parentH = heur->evaluate(*parent_ptr);
    if (parentH == -1) {
        assert(false);
        return;
    }
    switch(g_parameters.g_values) {
        case PlannerParameters::GTimestamp:
            parentG = getGt(parent_ptr);
            break;
        case PlannerParameters::GCost:
            parentG = getGc(parent_ptr);
            break;
        case PlannerParameters::GMakespan:
            parentG = getGm(parent_ptr);
            break;
        case PlannerParameters::GWeighted:
            parentG = g_parameters.g_weight * getGm(parent_ptr) + (1.0 - g_parameters.g_weight) * getGc(parent_ptr);
            break;
        default:
            assert(false);
    }

    parentF = parentG + parentH;
    for (; i < open_lists.size(); i++) {
        double childG, childH, childF;

        int numberOfChildrenWithSameG = 0;
        int numberOfChildren = 0;

        int index = i;
        if (mode == HIERARCHICAL) {
            if (is_g_plateau) {
                index = 1;
            } else {
                index = 0;
            }
        }
        vector<const Operator *>
            &ops =
            open_lists[index].only_preferred_operators ? preferred_operators
            : all_operators;
        for (int j = 0; j < ops.size(); j++) {
            assert(ops[j]->get_name().compare("wait") != 0);
            double maxTimeIncrement = 0.0;
            for (int k = 0; k < current_state.operators.size(); ++k) {
                maxTimeIncrement = max(maxTimeIncrement,
                        current_state.operators[k].time_increment);
            }
            double duration = ops[j]->get_duration(&current_state, 1);
            maxTimeIncrement = max(maxTimeIncrement, duration);
            double makeSpan = maxTimeIncrement + current_state.timestamp;
            TimedSymbolicStates timedSymbolicStates;
            if (ops[j]->is_applicable(*parent_ptr, timedSymbolicStates, false) &&
                    makeSpan < bestMakespan &&
                    (!g_parameters.use_tss_known || !tssKnown2(tcl,timedSymbolicStates))   // use_tss_known => !tssKnow2
               ) {
                TimeStampedState tss = TimeStampedState(current_state, *ops[j]);
                if(g_parameters.lazy_evaluation) {
                    childH = 42.0;   // set something != -1, this is NOT used below
                } else {
                    childH = heur->evaluate(tss);
                }
                if (childH == -1) {
                    continue;
                }

                switch(g_parameters.g_values) {
                    case PlannerParameters::GTimestamp:
                        childG = getGt(&tss);
                        break;
                    case PlannerParameters::GCost:
                        childG = getGc(parent_ptr, ops[j]);
                        break;
                    case PlannerParameters::GMakespan:
                        childG = getGm(&tss);
                        break;
                    case PlannerParameters::GWeighted:
                        childG = g_parameters.g_weight * getGm(&tss) 
                            + (1.0 - g_parameters.g_weight) * getGc(parent_ptr, ops[j]);
                        break;
                    default:
                        assert(false);
                }

                childF = childG + childH;
                numberOfChildren++;
                if (double_equals(parentG, childG)) {
                    childsWithSameG++;
                    numberOfChildrenWithSameG++;
                } else {
                    childsWithDifferentG++;
                }
                if (double_equals(parentF, childF)) {
                    childsWithSameF++;
                } else {
                    childsWithDifferentF++;
                }
                index = i;
                if (mode == HIERARCHICAL) {
                    if (double_equals(parentG, childG)) {
                        index = 1;
                    } else {
                        index = 0;
                    }
                }
                //               cout << "inserting " << ops[j]->get_name() << " with val: " << parentF << endl;
                double priority = g_parameters.lazy_evaluation ? parentF : childF;
                open_lists[index].open.push(std::tr1::make_tuple(parent_ptr,
                            ops[j], priority));
                generated_states++;
            }
        }
        TimeStampedState tss = current_state.let_time_pass(false);
        if(g_parameters.lazy_evaluation) {
            childH = 42.0;   // set something != -1, this is NOT used below
        } else {
            childH = heur->evaluate(tss);
        }
        if (childH == -1) {
            return;
        }

        switch(g_parameters.g_values) {
            case PlannerParameters::GTimestamp:
                childG = getGt(&tss);
                break;
            case PlannerParameters::GCost:
                childG = getGc(parent_ptr);
                break;
            case PlannerParameters::GMakespan:
                childG = getGm(&tss);
                break;
            case PlannerParameters::GWeighted:
                childG = g_parameters.g_weight * getGm(&tss) + (1.0 - g_parameters.g_weight) * getGc(parent_ptr);
                break;
            default:
                assert(false);
        }

        childF = childH + childG;
        numberOfChildren++;
        if (double_equals(parentG, childG)) {
            childsWithSameG++;
            numberOfChildrenWithSameG++;
        } else {
            childsWithDifferentG++;
        }
        if (double_equals(parentF, childF)) {
            childsWithSameF++;
        } else {
            childsWithDifferentF++;
        }

        if (numberOfChildrenWithSameG >= 2) {
            parentsWithTwoOrMoreChildsWithSameG++;
        } else {
            parentsWithAtMostOneChildWithSameG++;
        }
        if (numberOfChildren >= 2) {
            parentsWithTwoOrMoreChilds++;
        } else {
            parentsWithAtMostOneChild++;
        }
        double priority = g_parameters.lazy_evaluation ? parentF : childF;
        open_lists[i].open.push(std::tr1::make_tuple(parent_ptr,
                    g_let_time_pass, priority));
        generated_states++;
    }
}

enum SearchEngine::status BestFirstSearchEngine::fetch_next_state()
{
    OpenListInfo *open_info = select_open_queue();
    if(!open_info) {
        if(found_at_least_one_solution()) {
            cout << "Completely explored state space -- best plan found!" << endl;
            return SOLVED;
        }
        
        if(g_parameters.verbose) {
            time_t current_time = time(NULL);
            statistics(current_time);
        }
        cout << "Completely explored state space -- no solution!" << endl;
        return FAILED;
    }

    std::tr1::tuple<const TimeStampedState *, const Operator *, double> next =
        open_info->open.top();
    open_info->open.pop();

    const TimeStampedState* state = std::tr1::get<0>(next);
    const Operator* op = std::tr1::get<1>(next);

    TimedSymbolicStates tss;
    if (op != g_let_time_pass && !op->is_applicable(*state, tss, false)) {
        return fetch_next_state();
    }
    open_info->priority++;

    current_predecessor = std::tr1::get<0>(next);
    current_operator = std::tr1::get<1>(next);

    if(current_operator == g_let_time_pass) {
        // do not apply an operator but rather let some time pass until
        // next scheduled happening
        current_state = current_predecessor->let_time_pass();
    } else {
        assert(current_operator->get_name().compare("wait") != 0);
        current_state = TimeStampedState(*current_predecessor, *current_operator);
    }
    assert(&current_state != current_predecessor);
    return IN_PROGRESS;
}

OpenListInfo *BestFirstSearchEngine::select_open_queue()
{
    OpenListInfo *best = 0;

    if(mode == PRIORITY_BASED) {
        for(int i = 0; i < open_lists.size(); i++) {
            if(!open_lists[i].open.empty() && (best == 0 || open_lists[i].priority < best->priority))
                best = &open_lists[i];
        }
    } else if (mode == ROUND_ROBIN) {

        for (int i = 0; i < open_lists.size(); i++) {
            currentQueueIndex = (currentQueueIndex + 1) % open_lists.size();
            if (!open_lists[currentQueueIndex].open.empty()) {
                best = &open_lists[currentQueueIndex];
                break;
            }
        }

    } else {
        assert(mode == HIERARCHICAL);
        // Convention: open_lists[0] must be the f_m queue, and open_lists[1] must be the f_mw queue.
        // Requires: open_lists.size() == 2

        if (currentQueueIndex == -1)
            currentQueueIndex = 0;

        if (open_lists[currentQueueIndex].open.empty()) {
            currentQueueIndex = (currentQueueIndex + 1) % 2;
            if (!open_lists[currentQueueIndex].open.empty()) {
                best = &open_lists[currentQueueIndex];
            }
        } else {
            best = &open_lists[currentQueueIndex];
            assert(!open_lists[currentQueueIndex].open.empty());
        }
    }

    return best;
}
