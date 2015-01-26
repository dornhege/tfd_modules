#include "plannerParameters.h"
#include <iostream>
#if ROS_BUILD
#include <ros/ros.h>
#endif
#include <stdio.h>

PlannerParameters::PlannerParameters()
{
    // set defaults
    epsStateValueComparison = 0.000001;
    epsTimeComparison = 0.000001;
    epsDoubleComparison = 0.000001;

    epsSchedulingGapTime = 0.001;

    anytime_search = false;

    disallow_concurrent_actions = false;
    fetch_next_state_immediately_lets_time_pass = false;

    insert_let_time_pass_only_when_running_operators_not_empty = false;

    timeout_while_no_plan_found = 0;
    timeout_if_plan_found = 0;
    min_search_time_after_plan_found = 0;
    min_search_time_factor_after_plan_found = 0.0;

    greedy = false;
    lazy_evaluation = true;
    use_boosting = true;
    boost_strength = 1000;
    verbose = true;
    verbosePrintTime = 10.0;
    analyze = false;
    analyzeOutputNumericalFluents = false;
    analyzeCondensedOutput = true;
    analyzeDiscardedStatesByReason = true;
    analyzeLinkEqualStates = true;

    lazy_state_module_evaluation = -1;
    use_cost_modules_for_applicability = true;
    use_cost_modules_for_makespan_pruning = true;

    cyclic_cg_heuristic = false;
    cyclic_cg_preferred_operators = false;
    makespan_heuristic = false;
    makespan_heuristic_preferred_operators = false;
    no_heuristic = false;
    greedy_apply_heuristic = false;
    greedy_apply_heuristic_preferred_operators = false;
    greedy_apply_heuristic_max_depth = 0;

    cg_heuristic_zero_cost_waiting_transitions = true;
    cg_heuristic_fire_waiting_transitions_only_if_local_problems_matches_state = false;

    g_values = GTimestamp;
    g_weight = 0.5;

    queueManagementMode = BestFirstSearchEngine::PRIORITY_BASED;

    grounding_mode = GroundSingleReinsert;
    ground_n_max_groundings = 10;
    grounding_discount_mode = GroundingDiscountLinear;
    grounding_discount_gamma = 1.0;
    grounding_number_depends_on_state = true;

    use_known_by_logical_state_only = false;

    use_subgoals_to_break_makespan_ties = false;

    reschedule_plans = false;
    epsilonize_internally = false;
    epsilonize_externally = false;
    keep_original_plans = true;

    plan_name = "sas_plan";
    planMonitorFileName = "";

    monitoring_verify_timestamps = false;
}

PlannerParameters::~PlannerParameters()
{
}

bool PlannerParameters::readParameters(int argc, char** argv)
{
    bool ret = true;
    ret &= readROSParameters();
    ret &= readCmdLineParameters(argc, argv);

    if(!cyclic_cg_heuristic && !makespan_heuristic && !no_heuristic && !greedy_apply_heuristic) {
        if(planMonitorFileName.empty()) {   // for monitoring this is irrelevant
            cerr << "Error: you must select at least one heuristic!" << endl
                << "If you are unsure, choose options \"yY\" / cyclic_cg_heuristic." << endl;
            ret = false;
        }
    }
    if(timeout_if_plan_found < 0) {
        cerr << "Error: timeout_if_plan_found < 0, have: " << timeout_if_plan_found << endl;
        timeout_if_plan_found = 0;
        ret = false;
    }
    if(timeout_while_no_plan_found < 0) {
        cerr << "Error: timeout_while_no_plan_found < 0, have: " << timeout_while_no_plan_found << endl;
        timeout_while_no_plan_found = 0;
        ret = false;
    }
    if(min_search_time_after_plan_found < 0) {
        cerr << "Error: min_search_time_after_plan_found < 0, have: " << min_search_time_after_plan_found << endl;
        min_search_time_after_plan_found = 0;
        ret = false;
    }
    if(min_search_time_factor_after_plan_found < 0.0) {
        cerr << "Error: min_search_time_factor_after_plan_found < 0, have: " << min_search_time_factor_after_plan_found << endl;
        min_search_time_factor_after_plan_found = 0.0;
        ret = false;
    }
    if(use_known_by_logical_state_only) {
        cerr << "WARNING: known by logical state only is experimental and might lead to incompleteness!" << endl;
    }

    return ret;
}

void PlannerParameters::dump() const
{
    cout << endl << "Planner Paramters:" << endl;

    cout << "Eps State Value Comparison: " << epsStateValueComparison << endl;
    cout << "Eps Time Comparison: " << epsTimeComparison << endl;
    cout << "Eps Double Comparison: " << epsDoubleComparison << endl;
    cout << "Eps Scheduling Gap Time: " << epsSchedulingGapTime << endl;

    cout << "Anytime Search: " << (anytime_search ? "Enabled" : "Disabled") << endl;
    cout << "Disallow concurrent actions: " << (disallow_concurrent_actions ? "Enabled" : "Disabled") << endl;
    if(disallow_concurrent_actions)
        cout << "WARNING: planning non-temporally" << endl;

    cout << "fetch_next_state immediately lets time pass: " <<
        (fetch_next_state_immediately_lets_time_pass ? "Enabled" : "Disabled") << endl;
    if(fetch_next_state_immediately_lets_time_pass && !disallow_concurrent_actions)
        cout << "WARNING: planning temporally and fetch_next_state immediately lets_time_pass" << endl;
    cout << "Insert let_time_pass only when running operators is not empty: " <<
        (insert_let_time_pass_only_when_running_operators_not_empty ? "Enabled" : "Disabled") << endl;

    cout << "Timeout if plan was found: " << timeout_if_plan_found << " seconds";
    if(timeout_if_plan_found == 0)
        cout << " (no timeout)";
    cout << endl;
    cout << "Timeout while no plan was found: " << timeout_while_no_plan_found << " seconds";
    if(timeout_while_no_plan_found == 0)
        cout << " (no timeout)";
    cout << endl;
    cout << "Min search time after plan found: " << min_search_time_after_plan_found << " seconds" << endl;
    cout << "Min search time factor after plan found: " << min_search_time_factor_after_plan_found << " seconds" << endl;
    cout << "Greedy Search: " << (greedy ? "Enabled" : "Disabled") << endl;
    cout << "Verbose: " << (verbose ? "Enabled" : "Disabled") << endl;
    cout << "Verbose Print Time: " << verbosePrintTime << endl;
    cout << "Analyze: " << (analyze ? "Enabled" : "Disabled") << endl;
    cout << "Analyze Output NumericalFluents: " << (analyzeOutputNumericalFluents ? "Enabled" : "Disabled") << endl;
    cout << "Analyze Condensed Output: " << (analyzeCondensedOutput ? "Enabled" : "Disabled") << endl;
    cout << "Analyze Discarded States by Reason: " << (analyzeDiscardedStatesByReason ? "Enabled" : "Disabled") << endl;
    cout << "Analyze Link Equal States: " << (analyzeLinkEqualStates ? "Enabled" : "Disabled") << endl;

    cout << "Lazy Heuristic Evaluation: " << (lazy_evaluation ? "Enabled" : "Disabled") << endl;
    cout << "Boosting: " << (use_boosting ? "Enabled" : "Disabled") << endl;
    cout << "Boost Strength: " << boost_strength << endl;

    cout << "Lazy State Module Evaluation: " << lazy_state_module_evaluation;
    if(lazy_state_module_evaluation < 0)
        cout << " (auto)";
    cout << endl;
    cout << "Use cost modules for applicablity: "
        << (use_cost_modules_for_applicability ? "Enabled" : "Disabled") << endl;
    cout << "Use cost modules for makespan pruning: "
        << (use_cost_modules_for_makespan_pruning ? "Enabled" : "Disabled") << endl;

    cout << "Cyclic CG heuristic: " << (cyclic_cg_heuristic ? "Enabled" : "Disabled")
        << " \tPreferred Operators: " << (cyclic_cg_preferred_operators ? "Enabled" : "Disabled") << endl;
    cout << "Makespan heuristic: " << (makespan_heuristic ? "Enabled" : "Disabled")
        << " \tPreferred Operators: " << (makespan_heuristic_preferred_operators ? "Enabled" : "Disabled") << endl;
    cout << "No Heuristic: " << (no_heuristic ? "Enabled" : "Disabled") << endl;
    cout << "Greedy Apply heuristic: " << (greedy_apply_heuristic ? "Enabled" : "Disabled")
        << " \tPreferred Operators: " << (greedy_apply_heuristic_preferred_operators ? "Enabled" : "Disabled") << " Max Depth: " << greedy_apply_heuristic_max_depth << endl;
    cout << "Cg Heuristic Zero Cost Waiting Transitions: "
        << (cg_heuristic_zero_cost_waiting_transitions ? "Enabled" : "Disabled") << endl;
    cout << "Cg Heuristic Fire Waiting Transitions Only If Local Problems Matches State: "
        << (cg_heuristic_fire_waiting_transitions_only_if_local_problems_matches_state ? "Enabled" : "Disabled") << endl;

    cout << "GValues by: ";
    switch(g_values) {
        case GMakespan:
            cout << "Makespan";
            break;
        case GCost:
            cout << "Cost";
            break;
        case GTimestamp:
            cout << "Timestamp";
            break;
        case GWeighted:
            cout << "Weighted (" << g_weight << ")";
            break;
    }
    cout << endl;

    cout << "Queue management mode: ";
    switch(queueManagementMode) {
        case BestFirstSearchEngine::PRIORITY_BASED:
            cout << "Priority based";
            break;
        case BestFirstSearchEngine::ROUND_ROBIN:
            cout << "Round Robin";
            break;
    }
    cout << endl;

    cout << "Grounding mode: ";
    switch(grounding_mode) {
        case GroundAll:
            cout << "All";
            break;
        case GroundN:
            cout << "Ground N";
            break;
        case GroundSingleDiscard:
            cout << "Single and Discard";
            break;
        case GroundSingleReinsert:
            cout << "Single and Reinsert";
            break;
    }
    cout << endl;

    cout << "GroundN max groundings: " << ground_n_max_groundings << endl;

    cout << "Grounding Discount mode: ";
    switch(grounding_discount_mode) {
        case GroundingDiscountNone:
            cout << "None";
            break;
        case GroundingDiscountLinear:
            cout << "Linear";
            break;
        case GroundingDiscountExponential:
            cout << "Exponential";
            break;
    }
    cout << endl;
    cout << "Grounding Discount gamma: " << grounding_discount_gamma << endl;
    cout << "Grounding Number depends on state: " << (grounding_number_depends_on_state ? "true" : "false") << endl;

    cout << "Known by logical state only filtering: "
        << (use_known_by_logical_state_only ? "Enabled" : "Disabled") << endl;

    cout << "use_subgoals_to_break_makespan_ties: "
        << (use_subgoals_to_break_makespan_ties ? "Enabled" : "Disabled") << endl;

    cout << "Reschedule plans: " << (reschedule_plans ? "Enabled" : "Disabled") << endl;
    cout << "Epsilonize internally: " << (epsilonize_internally ? "Enabled" : "Disabled") << endl;
    cout << "Epsilonize externally: " << (epsilonize_externally ? "Enabled" : "Disabled") << endl;
    cout << "Keep original plans: " << (keep_original_plans ? "Enabled" : "Disabled") << endl;

    cout << "Plan name: \"" << plan_name << "\"" << endl;
    cout << "Plan monitor file: \"" << planMonitorFileName << "\"";
    if(planMonitorFileName.empty()) {
        cout << " (no monitoring)";
    }
    cout << endl;

    cout << "Monitoring verify timestamps: " << (monitoring_verify_timestamps ? "Enabled" : "Disabled") << endl;

    cout << endl;
}

bool PlannerParameters::readROSParameters()
{
    bool ret = true;

#if ROS_BUILD
    ros::NodeHandle nhPriv("~");

    nhPriv.param("eps_state_value_comparison", epsStateValueComparison, epsStateValueComparison);
    nhPriv.param("eps_time_comparison", epsTimeComparison, epsTimeComparison);
    nhPriv.param("eps_double_comparison", epsDoubleComparison, epsDoubleComparison);
    nhPriv.param("eps_scheduling_gap_time", epsSchedulingGapTime, epsSchedulingGapTime);

    nhPriv.param("anytime_search", anytime_search, anytime_search);
    nhPriv.param("disallow_concurrent_actions", disallow_concurrent_actions, disallow_concurrent_actions);
    nhPriv.param("insert_let_time_pass_only_when_running_operators_not_empty",
            insert_let_time_pass_only_when_running_operators_not_empty,
            insert_let_time_pass_only_when_running_operators_not_empty);
    nhPriv.param("fetch_next_state_immediately_lets_time_pass",
            fetch_next_state_immediately_lets_time_pass,
            fetch_next_state_immediately_lets_time_pass);

    nhPriv.param("timeout_if_plan_found", timeout_if_plan_found, timeout_if_plan_found);
    nhPriv.param("timeout_while_no_plan_found", timeout_while_no_plan_found, timeout_while_no_plan_found); 
    nhPriv.param("min_search_time_after_plan_found",
            min_search_time_after_plan_found, min_search_time_after_plan_found);
    nhPriv.param("min_search_time_factor_after_plan_found",
            min_search_time_factor_after_plan_found, min_search_time_factor_after_plan_found);

    nhPriv.param("analyze", analyze, analyze);
    nhPriv.param("analyze_output_numerical_fluents", analyzeOutputNumericalFluents,
            analyzeOutputNumericalFluents);
    nhPriv.param("analyze_condensed_output", analyzeCondensedOutput, analyzeCondensedOutput);
    nhPriv.param("analyze_discarded_states_by_reason", analyzeDiscardedStatesByReason, analyzeDiscardedStatesByReason);
    nhPriv.param("analyze_link_equal_states", analyzeLinkEqualStates, analyzeLinkEqualStates);

    nhPriv.param("greedy", greedy, greedy);
    nhPriv.param("lazy_evaluation", lazy_evaluation, lazy_evaluation);
    nhPriv.param("use_boosting", use_boosting, use_boosting);
    nhPriv.param("boost_strength", boost_strength, boost_strength);
    nhPriv.param("verbose_print_time", verbosePrintTime, verbosePrintTime);

    nhPriv.param("lazy_state_module_evaluation", lazy_state_module_evaluation, lazy_state_module_evaluation);
    nhPriv.param("use_cost_modules_for_applicability", use_cost_modules_for_applicability, use_cost_modules_for_applicability);
    nhPriv.param("use_cost_modules_for_makespan_pruning", use_cost_modules_for_makespan_pruning, use_cost_modules_for_makespan_pruning);

    nhPriv.param("cyclic_cg_heuristic", cyclic_cg_heuristic, cyclic_cg_heuristic);
    nhPriv.param("cyclic_cg_heuristic_preferred_operators", 
            cyclic_cg_preferred_operators, cyclic_cg_preferred_operators);
    nhPriv.param("makespan_heuristic", makespan_heuristic, makespan_heuristic);
    nhPriv.param("makespan_heuristic_preferred_operators", makespan_heuristic_preferred_operators, 
            makespan_heuristic_preferred_operators);
    nhPriv.param("greedy_apply_heuristic", greedy_apply_heuristic, greedy_apply_heuristic);
    nhPriv.param("greedy_apply_heuristic_preferred_operators",
            greedy_apply_heuristic_preferred_operators, greedy_apply_heuristic_preferred_operators);
    nhPriv.param("greedy_apply_heuristic_max_depth",
            greedy_apply_heuristic_max_depth, greedy_apply_heuristic_max_depth);
    nhPriv.param("no_heuristic", no_heuristic, no_heuristic);

    string gMode;
    if(nhPriv.getParam("g_values", gMode)) {
        if(gMode == "makespan") {
            g_values = GMakespan;
        } else if(gMode == "cost") {
            g_values = GCost;
        } else if(gMode == "timestamp") {
            g_values = GTimestamp;
        } else if(gMode == "weighted") {
            g_values = GWeighted;
            if(!nhPriv.getParam("g_weight", g_weight)) {
                ROS_FATAL("G mode weighted choosen, but g_weight not set.");
                ret = false;
            }
        } else {
            ROS_FATAL("Unknown value: %s for g_values. Valid values: [makespan, cost, timestamp, weighted]", gMode.c_str());
            ret = false;
        }
    }

    string qMode;
    if(nhPriv.getParam("queue_management_mode", qMode)) {
        if(qMode == "priority_based") {
            queueManagementMode = BestFirstSearchEngine::PRIORITY_BASED;
        } else if(qMode == "round_robin") {
            queueManagementMode = BestFirstSearchEngine::ROUND_ROBIN;
        } else {
            ROS_FATAL("Unknown value: %s for queue management mode. Valid values: [priority_based, round_robin, hierarchical]", qMode.c_str());
            ret = false;
        }
    }

    string groundingMode;
    if(nhPriv.getParam("grounding_mode", groundingMode)) {
        if(groundingMode == "ground_all") {
            grounding_mode = GroundAll;
        } else if(groundingMode == "ground_n") {
            grounding_mode = GroundN;
        } else if(groundingMode == "ground_single_discard") {
            grounding_mode = GroundSingleDiscard;
        } else if(groundingMode == "ground_single_reinsert") {
            grounding_mode = GroundSingleReinsert;
        } else {
            ROS_FATAL("Unknown value: %s for grounding mode: Valid values: [ground_all, ground_n, ground_single_discard, ground_single_reinsert]", groundingMode.c_str());
            ret = false;
        }
    }

    nhPriv.param("ground_n_max_groundings", ground_n_max_groundings, ground_n_max_groundings);

    string groundingDiscountMode;
    if(nhPriv.getParam("grounding_discount_mode", groundingDiscountMode)) {
        if(groundingDiscountMode == "none") {
            grounding_discount_mode = GroundingDiscountNone;
        } else if(groundingDiscountMode == "linear") {
            grounding_discount_mode = GroundingDiscountLinear;
        } else if(groundingDiscountMode == "exponential") {
            grounding_discount_mode = GroundingDiscountExponential;
        } else {
            ROS_FATAL("Unknown value: %s for grounding_discount mode: Valid values: [none, linear, exponential]", groundingDiscountMode.c_str());
            ret = false;
        }
    }

    nhPriv.param("grounding_discount_gamma", grounding_discount_gamma, grounding_discount_gamma);
    nhPriv.param("grounding_number_depends_on_state", grounding_number_depends_on_state, grounding_number_depends_on_state);

    nhPriv.param("use_known_by_logical_state_only",
            use_known_by_logical_state_only, use_known_by_logical_state_only);

    nhPriv.param("reschedule_plans", reschedule_plans, reschedule_plans);
    nhPriv.param("plan_name", plan_name, plan_name);

    // Don't get planMonitorFileName from param server as that is an input

    nhPriv.param("monitoring_verify_timestamps", monitoring_verify_timestamps, monitoring_verify_timestamps);
#endif

    return ret;
}

void PlannerParameters::printUsage() const
{
    printf("Usage: search <option characters>  (input read from stdin)\n");
    printf("Options are:\n");
    printf("  a - enable anytime search (otherwise finish on first plan found)\n");
    printf("  c - disallow any concurrent actions (plan classically, NOT temporal)\n");
    printf("  t <timeout secs> - total timeout in seconds for anytime search (when plan found)\n");
    printf("  T <timeout secs> - total timeout in seconds for anytime search (when no plan found)\n");
    printf("  m <monitor file> - monitor plan, validate a given plan\n");
    printf("  g - perform greedy search (follow heuristic)\n");
    printf("  l - disable lazy evaluation (Lazy = use parent's f instead of child's)\n");
    printf("  v - disable verbose printouts\n");
    printf("  y - cyclic cg CEA heuristic\n");
    printf("  Y - cyclic cg CEA heuristic - preferred operators\n");
    printf("  x - cyclic cg makespan heuristic \n");
    printf("  X - cyclic cg makespan heuristic - preferred operators\n");
    printf("  G [m|c|t|w] - G value evaluation, one of m - makespan, c - pathcost, t - timestamp, w [weight] - weighted / Note: One of those has to be set!\n");
    printf("  Q [r|p|h] - queue mode, one of r - round robin, p - priority, h - hierarchical\n");
    printf("  K - use tss known filtering (might crop search space)!\n");
    printf("  n - no_heuristic\n");
    printf("  r - reschedule_plans\n");
    printf("  e - epsilonize internally\n");
    printf("  f - epsilonize externally\n");
    printf("  p <plan file> - plan filename prefix\n");
    printf("  M v - monitoring: verify timestamps\n");
}

bool PlannerParameters::readCmdLineParameters(int argc, char** argv)
{
    for (int i = 1; i < argc; i++) {
        for (const char *c = argv[i]; *c != 0; c++) {
            if (*c == 'a') {
                anytime_search = true;
            } else if (*c == 'c') {
                disallow_concurrent_actions = true;
            } else if (*c == 't') {
                assert(i + 1 < argc);
                timeout_if_plan_found = atoi(string(argv[++i]).c_str());
            } else if (*c == 'T') {
                assert(i + 1 < argc);
                timeout_while_no_plan_found = atoi(string(argv[++i]).c_str());
            } else if (*c == 'm') {
                assert(i + 1 < argc);
                planMonitorFileName = string(argv[++i]);
            } else if (*c == 'g') {
                greedy = true;
            } else if (*c == 'l') {
                lazy_evaluation = false;
            } else if (*c == 'v') {
                verbose = false;
            } else if (*c == 'y') {
                cyclic_cg_heuristic = true;
            } else if (*c == 'Y') {
                cyclic_cg_preferred_operators = true;
            } else if (*c == 'x') {
                makespan_heuristic = true;
            } else if (*c == 'X') {
                makespan_heuristic_preferred_operators = true;
            } else if (*c == 'n') {
                no_heuristic = true;
            } else if (*c == 'G') {
                assert(i + 1 < argc);
                const char *g = argv[++i];
                if (*g == 'm') {
                    g_values = GMakespan;
                } else if (*g == 'c') {
                    g_values = GCost;
                } else if (*g == 't') {
                    g_values = GTimestamp;
                } else {
                    assert(*g == 'w');
                    g_values = GWeighted;
                    assert(i + 1 < argc);
                    g_weight = strtod(argv[++i], NULL);
                    assert(g_weight > 0 && g_weight < 1);  // for 0, 1 use G m or G c, others invalid.
                }
            } else if (*c == 'Q') {
                assert(i + 1 < argc);
                const char *g = argv[++i];
                if (*g == 'r') {
                    queueManagementMode = BestFirstSearchEngine::ROUND_ROBIN;
                } else {
                    assert(*g == 'p');
                    queueManagementMode = BestFirstSearchEngine::PRIORITY_BASED;
                }
            } else if (*c == 'K') {
                use_known_by_logical_state_only = true;
            } else if (*c == 'p') {
                assert(i + 1 < argc);
                plan_name = string(argv[++i]);
            } else if (*c == 'r') {
                reschedule_plans = true;
            } else if (*c == 'e') {
                epsilonize_internally = true;
            } else if (*c == 'f') {
                epsilonize_externally = true;
            } else if (*c == 'M') {
                assert(i + 1 < argc);
                const char *g = argv[++i];
                assert(*g == 'v');
                if (*g == 'v') {
                    monitoring_verify_timestamps = true;
                }
            } else {
                cerr << "Unknown option: " << *c << endl;
                printUsage();
                return false;
            }
        }
    }

    return true;
}

