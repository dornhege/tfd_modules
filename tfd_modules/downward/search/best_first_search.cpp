#include "best_first_search.h"

#include "globals.h"
#include "heuristic.h"
#include "successor_generator.h"
#include "plannerParameters.h"
#include "analysis.h"
#include <time.h>
#include <iomanip>
#include "ros_printouts.h"
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH
#include <fstream>

#include <cassert>
#include <cmath>

using namespace std;

static const bool DEBUG_GENERATE_SUCCESSORS = false;
static const bool RECORD_MEM_USAGE_STATS = true;

OpenListInfo::OpenListInfo(Heuristic *heur, bool only_pref)
{
    heuristic = heur;
    only_preferred_operators = only_pref;
    priority = 0;
}

BestFirstSearchEngine::BestFirstSearchEngine(QueueManagementMode _mode) :
    current_state(*g_initial_state), currentQueueIndex(-1), mode(_mode)
{
    current_predecessor = 0;
    current_operator = 0;
    start_time = time(NULL);
    bestMakespan = HUGE_VAL;
    bestSumOfGoals = HUGE_VAL;
}

BestFirstSearchEngine::~BestFirstSearchEngine()
{
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
        preferred_operator_heuristics.push_back(heuristic);
        open_lists.push_back(OpenListInfo(heuristic, true));
    }
}

void BestFirstSearchEngine::initialize()
{
    last_stat_time = time(NULL);
    assert(!open_lists.empty());
}

void BestFirstSearchEngine::statistics(time_t & current_time)
{
    cout << endl;
    cout << "Search Time: " << (current_time - start_time) << " sec." << endl;
    if(RECORD_MEM_USAGE_STATS) {
        pid_t pid = getpid();
        char stat_file[1024];
        snprintf(stat_file, 1023, "/proc/%d/stat", pid);
        std::ifstream stat(stat_file, std::ifstream::in);
        if(stat.good()) {
            unsigned int VSIZE = 22;
            std::string tmp;
            for(unsigned int i = 0; i < VSIZE; i++) {
                stat >> tmp;
            }
            unsigned int vsize;
            stat >> vsize;
            cout << "Virtual Memory: " << vsize/1024 << " kb" << endl;
            stat.close();   // FIXME check for hangs
        }
    }
    search_statistics.dump(closed_list.size(), current_time);
    cout << "OpenList sizes:";
    for(unsigned int i = 0; i < open_lists.size(); ++i) {
        cout << " " << open_lists[i].open.size();
    }
    cout << endl;
    cout << "Heuristic Computations (per heuristic):";
    unsigned long totalHeuristicComputations = 0;
    for(unsigned int i = 0; i < heuristics.size(); ++i) {
        Heuristic *heur = heuristics[i];
        totalHeuristicComputations += heur->get_num_computations();
        cout << " " << heur->get_num_computations();
    }
    cout << " Total: " << totalHeuristicComputations << endl;
    cout << "Heuristic calculation times (per heuristic):" << endl;
    for(unsigned int i = 0; i < heuristics.size(); ++i) {
        Heuristic *heur = heuristics[i];
        heur->printTimingStats();
    }
    cout << "Best heuristic value: " << best_heuristic_values[0] << endl << endl;
    //    cout << "Best state:" << endl;
    //    const TimeStampedState &state = *best_states[0];
    //    if(&state) {
    //        dump_plan_prefix_for__state(state);
    //        best_states[0]->dump();
    //    }
    //    cout << endl;
    if(g_parameters.analyze && !g_analysis.writeDot("state_space")) {
        ROS_ERROR("Failed to write Analysis to state_space...");
    }
}

void BestFirstSearchEngine::dump_transition() const
{
    cout << endl;
    if(current_predecessor != 0) {
        cout << "DEBUG: current predecessor is: " << endl;
        current_predecessor->dump(g_parameters.verbose);
    }
    cout << "DEBUG: current operator is: ";
    if(current_operator != 0) {
        current_operator->dump();
    } else {
        cout << "No operator before initial state." << endl;
    }
    cout << "DEBUG: current state is: " << endl;
    current_state.dump(g_parameters.verbose);
    cout << endl;
}

void BestFirstSearchEngine::dump_everything() const
{
    dump_transition();
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
            std::tr1::get<0>(*it2)->dump(true);
            cout << "op" << endl;
            std::tr1::get<1>(*it2)->dump();
            double h = std::tr1::get<2>(*it2);
            cout << "Value: " << h << endl;
            cout << "end OpenListEntry" << endl;
        }
    }
    cout << "Closed List" << endl;
    closed_list.dump();
    cout << endl;
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

    // when using subgoals we want to keep states with the same makespan as they might have better subgoals
    if(g_parameters.use_subgoals_to_break_makespan_ties) {
        if (makeSpan <= bestMakespan && !closed_list.contains(current_state)) {
            discard = false;
        }
    } else {
        if (makeSpan < bestMakespan && !closed_list.contains(current_state)) {
            discard = false;
        }
    }

    // throw away any states resulting from zero cost actions (can't handle)
    if(current_predecessor && current_operator && current_operator != g_let_time_pass &&
            current_operator->get_duration(current_predecessor, false) <= 0.0) {
        discard = true;
    }

    if(!discard) {
        const TimeStampedState *parent_ptr = closed_list.insert(current_state,
                current_predecessor, current_operator);
        assert(&current_state != current_predecessor);
        g_analysis.recordClosingStep(current_predecessor, current_operator, parent_ptr);

        // evaluate the current/parent state
        // also do this for non/lazy-evaluation as the heuristics
        // provide preferred operators and check_goal/check_progress rely
        // on the correct value for get_heuristic anyways!
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
    } else {
        g_analysis.recordDiscardingStep(current_predecessor, current_operator, current_state, closed_list);
        if ((current_operator == g_let_time_pass) &&
                current_state.operators.empty() &&
                makeSpan < bestMakespan) {
            // arrived at same state by letting time pass
            // e.g. when having an action with duration and only start effect
            // the result would be discarded, check if we are at goal    
            for(int i = 0; i < heuristics.size(); i++) {
                heuristics[i]->evaluate(current_state);
            }
            if(!is_dead_end()) {
                if(check_goal())
                    return SOLVED;
            }
        }
    }

    time_t current_time = time(NULL);
    if(g_parameters.verbose && current_time - last_stat_time >= g_parameters.verbosePrintTime) {
        statistics(current_time);
        last_stat_time = current_time;
    }

    // use different timeouts depending if we found a plan or not.
    if(found_solution()) {
        static time_t first_solution_time = current_time;
        // first check: timeout if plan found
        if(g_parameters.timeout_if_plan_found > 0 
                && current_time - start_time > g_parameters.timeout_if_plan_found) {
            // second check: timeout from first plan found to min_search_time_after_plan_found
            // only do this, if the extra time is request, otherwise we ran into the timeout
            if(g_parameters.min_search_time_after_plan_found <= 0
                    && g_parameters.min_search_time_factor_after_plan_found <= 0) {
                ROS_INFO("Ran into timeout_if_plan_found");
                if(g_parameters.verbose)
                    statistics(current_time);
                return SOLVED_TIMEOUT;
            } else {
                int extra_timeout = g_parameters.min_search_time_after_plan_found;
                int proportion_extra =
                    int(g_parameters.min_search_time_factor_after_plan_found
                            * double(first_solution_time - start_time));
                if(proportion_extra > extra_timeout)
                    extra_timeout = proportion_extra;

                if(current_time - first_solution_time > extra_timeout) {
                    ROS_INFO("Ran into min_search_time_(factor_)after_plan_found");
                    if(g_parameters.verbose)
                        statistics(current_time);
                    return SOLVED_TIMEOUT;
                }
            }
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
    // pick an arbitrary one. Heuristics are assumed to not miss a goal
    // state and especially not to report a goal as dead end.
    Heuristic *heur = open_lists[0].heuristic;
 
    // We actually need this silly !heur->is_dead_end() check because
    // this state *might* be considered a non-dead end by the
    // overall search even though heur considers it a dead end
    // (e.g. if heur is the CG heuristic, but the FF heuristic is
    // also computed and doesn't consider this state a dead end.
    // If heur considers the state a dead end, it cannot be a goal
    // state (heur will not be *that* stupid). We may not call
    // get_heuristic() in such cases because it will barf.
    if(!heur->is_dead_end() && heur->get_heuristic() == 0) {
        if(current_state.operators.size() > 0) {
            return false;
        }
        // found goal

        if(!current_state.satisfies(g_goal)) {  // will assert...
            dump_everything();
        }
        assert(current_state.operators.empty() && current_state.satisfies(g_goal));

        g_analysis.recordGoal(current_state);

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
    if(!g_parameters.use_boosting)
        return;

    // Boost the "preferred operator" open lists somewhat whenever
    // progress is made. This used to be used in multi-heuristic mode
    // only, but it is also useful in single-heuristic mode, at least
    // in Schedule.
    //
    // Future Work: Test the impact of this, and find a better way of rewarding
    // successful exploration. For example, reward only the open queue
    // from which the good state was extracted and/or the open queues
    // for the heuristic for which a new best value was found.

    for(int i = 0; i < open_lists.size(); i++)
        if(open_lists[i].only_preferred_operators)
            open_lists[i].priority -= g_parameters.boost_strength;
}

bool knownByLogicalStateOnly(LogicalStateClosedList& scl, const TimedSymbolicStates& timedSymbolicStates)
{
    // feature disabled -> return false = state unkown -> insert
    if(!g_parameters.use_known_by_logical_state_only)
       return false;
    assert(timedSymbolicStates.size() > 0);
    bool ret = true;
    for (int i = 0; i < timedSymbolicStates.size(); ++i) {
        if (scl.count(timedSymbolicStates[i].first) > 0) {
            double currentBestMakespan = scl[timedSymbolicStates[i].first];
            if (timedSymbolicStates[i].second + g_parameters.epsTimeComparison < currentBestMakespan) {
                ret = false;
                scl[timedSymbolicStates[i].first] = timedSymbolicStates[i].second;
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
    g_successor_generator->generate_applicable_ops(*parent_ptr, all_operators);
    // Filter ops that cannot be applicable just from the preprocess data (doesn't guarantee full applicability)

    if(DEBUG_GENERATE_SUCCESSORS) {
        cout << endl << "GENERATE_SUCCESSORS" << endl;
        if(all_operators.empty())
            cout << "all_operators empty." << endl;
        else {
            cout << "all_operators:" << endl;
            forEach(const Operator* op, all_operators) {
                op->dump();
            }
        }
        fflush(stdout);
    }

    vector<const Operator *> preferred_operators;
    for(int i = 0; i < preferred_operator_heuristics.size(); i++) {
        Heuristic *heur = preferred_operator_heuristics[i];
        if(!heur->is_dead_end()) {
            heur->get_preferred_operators(preferred_operators);
        }
    }

    vector<const Operator *> preferred_applicable_operators;
    // all_operators contains a superset of applicable operators based on preprocess data
    // this in not necessarily all operators in the task, but also doesn't guarantee applicablity, yet.
    // Here we split this superset of operators in two:
    // - preferred_applicable_operators will be those operators from all_operators that are preferred_operators
    // - all_operators will be the rest (i.e. the non-preferred operators)
    // preferred_applicable_operators thus contains all operators that are preferred and might be applicable,
    // i.e. a subset of preferred_operators
    // The preferred_operators to be used will thus be only the preferred_applicable_operators
    for(int k = 0; k < preferred_operators.size(); ++k) {
        for(int l = 0; l < all_operators.size(); ++l) {
            if(all_operators[l] == preferred_operators[k]) {
                all_operators[l] = all_operators[all_operators.size() - 1];
                all_operators.pop_back();
                preferred_applicable_operators.push_back(preferred_operators[k]);
                break;
            }
        }
    }
    preferred_operators = preferred_applicable_operators;

    bool lazy_state_module_eval = g_parameters.lazy_state_module_evaluation > 0;

    // compute expected min makespan of this op
    double maxTimeIncrement = 0.0;
    for(int k = 0; k < parent_ptr->operators.size(); ++k) {
        maxTimeIncrement = max(maxTimeIncrement, parent_ptr->operators[k].time_increment);
    }

    // check if we can prune here by comparing the min timestamp this op will reach
    // and the best plan we have so far
    double makespan = maxTimeIncrement + parent_ptr->timestamp;
    bool betterMakespan = makespan < bestMakespan;
    if(g_parameters.use_subgoals_to_break_makespan_ties && makespan == bestMakespan)
        betterMakespan = true;
    if(!betterMakespan) // parent state is already past our best plan - skip
        return;

    for(int i = 0; i < open_lists.size(); i++) {
        if(DEBUG_GENERATE_SUCCESSORS)
            cout << "OPEN LIST: " << i << endl;
        Heuristic *heur = open_lists[i].heuristic;

        double priority = -1;   // invalid
        // lazy eval = compute priority by parent
        if(g_parameters.lazy_evaluation) {
            double parentG = getG(parent_ptr, parent_ptr, NULL);
            double parentH = heur->get_heuristic();
            assert(!heur->is_dead_end());
            double parentF = parentG + parentH;
            if(g_parameters.greedy)
                priority = parentH;
            else
                priority = parentF;
        }

        OpenList &open = open_lists[i].open;
        vector<const Operator *> & ops =
            open_lists[i].only_preferred_operators ? preferred_operators : all_operators;

        // push successors from applicable ops
        for(int j = 0; j < ops.size(); j++) {
            assert(ops[j]->get_name().compare("wait") != 0);
            if(DEBUG_GENERATE_SUCCESSORS) {
                cout << endl << "Testing op:" << endl;
                ops[j]->dump();
            }

            if(ops[j]->isGrounded()) {
                insert_successor(ops[j], open_lists[i], i, parent_ptr, priority, maxTimeIncrement);
            } else {
                // decide what to do with an ungrounded op.
                if(g_parameters.grounding_mode == PlannerParameters::GroundAll) {
                    // ground the ungrounded one as often as we can,
                    // insert the grounded ones,
                    // discard the ungrounded one
                    bool couldGround = true;
                    while(couldGround) {
                        couldGround = false;
                        Operator opGround = ops[j]->ground(*parent_ptr, false, couldGround);
                        if(!couldGround)
                            break;
                        pair<set<Operator>::iterator, bool> ret = g_grounded_operators.insert(opGround);
                        // we want to work with the it, not the opGround, which will go out of scope
                        // it points to the Op actually inside g_grounded_operators
                        // Also, if the same grounded op already exists, this will point to it, not
                        // some new one.
                        const Operator* groundedOp = &(*ret.first);
                        insert_successor(groundedOp, open_lists[i], i,
                                parent_ptr, priority, maxTimeIncrement);
                    }
                } else if(g_parameters.grounding_mode == PlannerParameters::GroundN) {
                    // ground the ungrounded one max N times.
                    // insert the grounded ones,
                    // discard the ungrounded one
                    for(int g = 0; g < g_parameters.ground_n_max_groundings; g++) {
                        bool couldGround = false;
                        Operator opGround = ops[j]->ground(*parent_ptr, false, couldGround);
                        if(!couldGround)
                            break;
                        pair<set<Operator>::iterator, bool> ret = g_grounded_operators.insert(opGround);
                        // we want to work with the it, not the opGround, which will go out of scope
                        // it points to the Op actually inside g_grounded_operators
                        // Also, if the same grounded op already exists, this will point to it, not
                        // some new one.
                        const Operator* groundedOp = &(*ret.first);
                        insert_successor(groundedOp, open_lists[i], i,
                                parent_ptr, priority, maxTimeIncrement);
                    }
                } else if(g_parameters.grounding_mode == PlannerParameters::GroundSingleDiscard) {
                    // ground the ungrounded one once (if poss)
                    // insert the grounded one,
                    // discard the ungrounded one
                    bool couldGround = false;
                    Operator opGround = ops[j]->ground(*parent_ptr, false, couldGround);
                    if(couldGround) {
                        pair<set<Operator>::iterator, bool> ret = g_grounded_operators.insert(opGround);
                        const Operator* groundedOp = &(*ret.first);
                        insert_successor(groundedOp, open_lists[i], i,
                                parent_ptr, priority, maxTimeIncrement);
                    }
                } else if(g_parameters.grounding_mode == PlannerParameters::GroundSingleReinsert) {
                    // Actually insert the ungrounded op in the open list
                    // fetch_next_state needs to deal with that correctly.
                    if(g_parameters.lazy_evaluation)    // ensure that we won't have to recompute the
                        ROS_ASSERT(priority >= 0);      // priority in lazy_evaluation mode, which 
                                                        // would invalidate calls to get_heuristic
                    insert_ungrounded_successor(ops[j], open_lists[i], i,
                            parent_ptr, priority, maxTimeIncrement);
                }
            }
        }

        if(DEBUG_GENERATE_SUCCESSORS) cout << "Adding let time pass." << endl;

        // Inserted all children, now insert one more child by letting time pass
        // only allow let_time_pass if there are running operators (i.e. there is time to pass)
        if(!g_parameters.insert_let_time_pass_only_when_running_operators_not_empty || !parent_ptr->operators.empty()) {
            // non lazy eval = compute priority by child
            if(!g_parameters.lazy_evaluation) {
                // compute child
                TimeStampedState tss = parent_ptr->let_time_pass(false, true, lazy_state_module_eval);
                double childG = getG(&tss, parent_ptr, NULL);
                double childH = heur->evaluate(tss);
                if(heur->is_dead_end()) {
                    continue;
                }

                double childF = childH + childG;
                if(g_parameters.greedy)
                    priority = childH;
                else
                    priority = childF;
            }
            open.push(std::tr1::make_tuple(parent_ptr, g_let_time_pass, priority));
            g_analysis.recordOpenPush(parent_ptr, g_let_time_pass, i, priority);
            search_statistics.countChild(i);
        }
    }
    search_statistics.finishExpansion();
    if(DEBUG_GENERATE_SUCCESSORS) cout << "Generated successors." << endl;
}

void BestFirstSearchEngine::insert_successor(const Operator* op, OpenListInfo& openInfo, int openIndex,
        const TimeStampedState* parent_ptr, double priority, double maxParentTimeIncrement)
{
    if(DEBUG_GENERATE_SUCCESSORS) {
        cout << endl << "insert_successor: op:" << endl;
        op->dump();
    }
    ROS_ASSERT(op->isGrounded());

    Heuristic* heur = openInfo.heuristic;
    OpenList & open = openInfo.open;

    bool lazy_state_module_eval = g_parameters.lazy_state_module_evaluation > 0;

    // compute expected min makespan of this op
    if(DEBUG_GENERATE_SUCCESSORS) cout << "Getting Duration..." << endl;
    double duration = op->get_duration(parent_ptr, lazy_state_module_eval,
            g_parameters.use_cost_modules_for_makespan_pruning);
    if(DEBUG_GENERATE_SUCCESSORS) cout << "Duration: " << duration << endl;
    double maxTimeIncrement = max(maxParentTimeIncrement, duration);
    double makespan = maxTimeIncrement + parent_ptr->timestamp;
    bool betterMakespan = makespan < bestMakespan;
    if(g_parameters.use_subgoals_to_break_makespan_ties && makespan == bestMakespan)
        betterMakespan = true;

    // Generate a child/Use an operator if
    // - it is applicable
    // - its minimum makespan is better than the best we had so far
    // - if knownByLogicalStateOnly hasn't closed this state (when feature enabled)

    // only compute tss if needed
    TimedSymbolicStates timedSymbolicStates;
    TimedSymbolicStates* tssPtr = NULL;
    if(g_parameters.use_known_by_logical_state_only)
        tssPtr = &timedSymbolicStates;

    if(DEBUG_GENERATE_SUCCESSORS) cout << "Checking applicability..." << endl;
    // use_modules here only, when lazy_state_module_eval is off
    // otherwise we get problems in effect application
    if(betterMakespan && op->is_applicable(*parent_ptr, lazy_state_module_eval, tssPtr,
                lazy_state_module_eval == 0) &&
            (!knownByLogicalStateOnly(logical_state_closed_list, timedSymbolicStates))) {
        if(DEBUG_GENERATE_SUCCESSORS) cout << "Applicable" << endl;
        // non lazy eval = compute priority by child
        if(!g_parameters.lazy_evaluation) {
            // need to compute the child to evaluate it
            TimeStampedState tss = TimeStampedState(*parent_ptr, *op, lazy_state_module_eval);
            double childG = getG(&tss, parent_ptr, op);
            double childH = heur->evaluate(tss);
            if(heur->is_dead_end())
                return;
            double childF = childG + childH;
            if(g_parameters.greedy)
                priority = childH;
            else
                priority = childF;
        }

        open.push(std::tr1::make_tuple(parent_ptr, op, priority));
        g_analysis.recordOpenPush(parent_ptr, op, openIndex, priority);
        search_statistics.countChild(openIndex);
    }
}

void BestFirstSearchEngine::insert_ungrounded_successor(const Operator* op, OpenListInfo& openInfo,
        int openIndex,
        const TimeStampedState* parent_ptr, double priority, double maxParentTimeIncrement)
{
    if(DEBUG_GENERATE_SUCCESSORS) {
        cout << endl << "insert_ungrounded_successor: op:" << endl;
        op->dump();
    }
    ROS_ASSERT(!op->isGrounded());

    Heuristic* heur = openInfo.heuristic;
    OpenList & open = openInfo.open;

    bool lazy_state_module_eval = g_parameters.lazy_state_module_evaluation > 0;

    // compute expected min makespan of this op
    if(DEBUG_GENERATE_SUCCESSORS) cout << "Getting Duration..." << endl;
    // this could still give us some info, but we can't allow modules on the ungrounded op.
    double duration = op->get_duration(parent_ptr, lazy_state_module_eval, false);
    if(DEBUG_GENERATE_SUCCESSORS) cout << "Duration: " << duration << endl;
    double maxTimeIncrement = max(maxParentTimeIncrement, duration);
    double makespan = maxTimeIncrement + parent_ptr->timestamp;
    bool betterMakespan = makespan < bestMakespan;
    if(g_parameters.use_subgoals_to_break_makespan_ties && makespan == bestMakespan)
        betterMakespan = true;

    // Generate a child/Use an operator if
    // - it is applicable
    // - its minimum makespan is better than the best we had so far
    // - if knownByLogicalStateOnly hasn't closed this state (when feature enabled)

    // only compute tss if needed
    TimedSymbolicStates timedSymbolicStates;
    TimedSymbolicStates* tssPtr = NULL;
    if(g_parameters.use_known_by_logical_state_only)
        tssPtr = &timedSymbolicStates;

    if(DEBUG_GENERATE_SUCCESSORS) cout << "Checking applicability..." << endl;
    // TODO can stop here at No better makespan (otherwise should have never been reinserted)
    // Is this obvious somehow? can we see this in grounded, then not liveGroundDiscard/closed/discarded
    // Timestmap??
    // Should be similar like a discard that stops...
    //
    // check applicability, but do not use modules for that.
    if(betterMakespan && op->is_applicable(*parent_ptr, lazy_state_module_eval, tssPtr, false) &&
            (!knownByLogicalStateOnly(logical_state_closed_list, timedSymbolicStates))) {
        if(DEBUG_GENERATE_SUCCESSORS) cout << "Applicable" << endl;

        // base priority is determined by the parent state as we cannot
        // produce a child (would need to ground for that)
        // when in non-lazy eval the parent prior might not have been determined,
        // do that now.
        if(priority < 0) {
            double parentG = getG(parent_ptr, parent_ptr, NULL);
            double parentH = heur->evaluate(*parent_ptr);
            assert(!heur->is_dead_end());
            double parentF = parentG + parentH;
            if(g_parameters.greedy)
                priority = parentH;
            else
                priority = parentF;
        }
        if(g_parameters.grounding_mode == PlannerParameters::GroundSingleReinsert) {
            double discount = 1.0;
            switch(g_parameters.grounding_discount_mode) {
                case PlannerParameters::GroundingDiscountLinear:
                    discount = 1.0 +
                        g_parameters.grounding_discount_gamma * op->getNumBranches(parent_ptr);
                    break;
                case PlannerParameters::GroundingDiscountExponential:
                    discount = 
                        pow(g_parameters.grounding_discount_gamma, op->getNumBranches(parent_ptr));
                    break;
                case PlannerParameters::GroundingDiscountNone:
                    discount = 1.0;
                    break;
            }
            priority *= discount;
        }

        open.push(std::tr1::make_tuple(parent_ptr, op, priority));
        g_analysis.recordOpenPush(parent_ptr, op, openIndex, priority);
        search_statistics.countLiveBranch(openIndex);
    } else {
        g_analysis.recordLiveGroundingUngroundedDiscard(parent_ptr, op);
    }
}

enum SearchEngine::status BestFirstSearchEngine::fetch_next_state()
{
    OpenListInfo *open_info = select_open_queue();
    if(!open_info) {
        if(found_at_least_one_solution()) {
            cout << "Completely explored state space -- best plan found!" << endl;
            return SOLVED_COMPLETE;
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
    open_info->priority++;

    const TimeStampedState* open_state = std::tr1::get<0>(next);
    const Operator* open_op = std::tr1::get<1>(next);

    // grounding check if ungrounded or in other grounding mode...
    if(g_parameters.grounding_mode != PlannerParameters::GroundSingleReinsert) {
        ROS_ASSERT(open_op->isGrounded());   // Only GroundSingleReinsert is allowed to push ungrounded ops
    } else {
        // op might be ungrounded, deal with that.
        if(!open_op->isGrounded()) {
            // check/force stats here as this might take a while
            time_t current_time = time(NULL);
            if(g_parameters.verbose && current_time - last_stat_time >= g_parameters.verbosePrintTime) {
                statistics(current_time);
                last_stat_time = current_time;
            }

            bool couldGround = false;
            Operator opGround = open_op->ground(*open_state, false, couldGround);
            if(couldGround) {
                pair<set<Operator>::iterator, bool> ret = g_grounded_operators.insert(opGround);
                g_analysis.recordLiveGrounding(open_state, &(*ret.first));
                // reinsert (only) if we could ground
                // recover index
                int openIndex = -1;
                for(int i = 0; i < open_lists.size(); i++) {
                    if(&open_lists[i] == open_info) {
                        openIndex = i;
                        break;
                    }
                }
                if(g_parameters.ground_n_max_groundings > 0) {
                    if(open_op->getNumBranches(open_state) < g_parameters.ground_n_max_groundings) {
                        insert_ungrounded_successor(open_op, *open_info, openIndex,
                                open_state, -1.0, 0.0);
                    } else {
                        // grounded out.
                        g_analysis.recordLiveGroundingGroundedOut(open_state, open_op);
                    }
                } else {
                    insert_ungrounded_successor(open_op, *open_info, openIndex,
                            open_state, -1.0, 0.0);
                }

                // now we can re set open_op to the grounded one.
                open_op = &(*ret.first);    // open_op is the grounded one, not the
                if(!open_op->is_applicable(*open_state, false)) {
                    g_analysis.recordLiveGroundingDiscard(open_state, open_op);
                    // OK, we had a grounding, but that didn't work. As we can't produce a successor,
                    // just discard this one and continue looking.
                    return fetch_next_state();
                }
            } else {    // could not ground this one, so we can't produce a successor, just discard...
                g_analysis.recordLiveGroundingGroundedOut(open_state, open_op);
                return fetch_next_state();
            }
        }
    }

    if(g_parameters.lazy_state_module_evaluation > 0) {
        // tentative new current_predecessor and current_operator
        // We need to recheck operator applicability in case a lazy evaluated op (relaxed module calls)
        // was inserted in the open queue
        if (open_op != g_let_time_pass && !open_op->is_applicable(*open_state, false)) {
            g_analysis.recordModuleRelaxedDiscardingStep(open_state, open_op);
            return fetch_next_state();
        }
    }

    current_predecessor = open_state;
    current_operator = open_op;

    if(current_operator == g_let_time_pass) {
        // do not apply an operator but rather let some time pass until
        // next scheduled happening
        current_state = current_predecessor->let_time_pass(false, true, false);
    } else {
        assert(current_operator->get_name().compare("wait") != 0);
        current_state = TimeStampedState(*current_predecessor, *current_operator, false);
        if(g_parameters.fetch_next_state_immediately_lets_time_pass) {
            // FIXME This is OK regarding closed list, plan tracing, etc. as we here define
            // what the successor (current_state) of current_predecessor/op actually is.
            while(!current_state.operators.empty()) {
                current_state = current_state.let_time_pass(false, true, false);
            }
        }
    }
    assert(&current_state != current_predecessor);
    return IN_PROGRESS;
}

OpenListInfo *BestFirstSearchEngine::select_open_queue()
{
    OpenListInfo *best = 0;

    switch(mode) {
        case PRIORITY_BASED:
            for(int i = 0; i < open_lists.size(); i++) {
                if(!open_lists[i].open.empty() && (best == 0 || open_lists[i].priority < best->priority))
                    best = &open_lists[i];
            }
            break;
        case ROUND_ROBIN:
            for(int i = 0; i < open_lists.size(); i++) {
                currentQueueIndex = (currentQueueIndex + 1) % open_lists.size();
                if(!open_lists[currentQueueIndex].open.empty()) {
                    best = &open_lists[currentQueueIndex];
                    break;
                }
            }
            break;
    }

    return best;
}

double BestFirstSearchEngine::getGc(const TimeStampedState *state) const
{
    return closed_list.getCostOfPath(*state);
}

double BestFirstSearchEngine::getGc(const TimeStampedState *state,
        const Operator *op) const
{
    double opCost = 0.0;
    if (op && op != g_let_time_pass) {
        opCost += op->get_duration(state, false);
    }
    return getGc(state) + opCost;
}

double BestFirstSearchEngine::getGm(const TimeStampedState *state) const
{
    double longestActionDuration = 0.0;
    for (int i = 0; i < state->operators.size(); ++i) {
        const ScheduledOperator* op = &state->operators[i];
        double duration = 0.0;
        if (op && op != g_let_time_pass) {
            duration = op->get_duration(state, false);
        }
        if (duration > longestActionDuration) {
            longestActionDuration = duration;
        }
    }
    return state->timestamp + longestActionDuration;
}

double BestFirstSearchEngine::getGt(const TimeStampedState *state) const
{
    return state->timestamp;
}

/**
 * If mode is cost or weighted a parent_ptr and op have to be given.
 *
 * \param [in] state_ptr the state to compute G for
 * \param [in] closed_ptr should be a closed node that op could be applied to, if state is not closed (i.e. a child)
 */
double BestFirstSearchEngine::getG(const TimeStampedState* state_ptr, 
        const TimeStampedState* closed_ptr, const Operator* op) const
{
    double g = HUGE_VAL;
    switch(g_parameters.g_values) {
        case PlannerParameters::GTimestamp:
            g = getGt(state_ptr);
            break;
        case PlannerParameters::GCost:
            if(op == NULL)
                g = getGc(closed_ptr);
            else
                g = getGc(closed_ptr, op);
            break;
        case PlannerParameters::GMakespan:
            g = getGm(state_ptr);
            break;
        case PlannerParameters::GWeighted:
            if(op == NULL)
                g = g_parameters.g_weight * getGm(state_ptr) 
                    + (1.0 - g_parameters.g_weight) * getGc(closed_ptr);
            else
                g = g_parameters.g_weight * getGm(state_ptr) 
                    + (1.0 - g_parameters.g_weight) * getGc(closed_ptr, op);
            break;
        default:
            assert(false);
    }
    return g;
}

