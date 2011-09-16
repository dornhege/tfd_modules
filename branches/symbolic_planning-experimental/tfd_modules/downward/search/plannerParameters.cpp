#include "plannerParameters.h"
#include <iostream>
#if ROS_BUILD
#include <ros/ros.h>
#endif

PlannerParameters::PlannerParameters()
{
    // set defaults
    anytime_search = false;
    timeout_while_no_plan_found = 0;
    timeout_if_plan_found = 0;

    greedy = false;
    lazy_evaluation = true;

    cyclic_cg_heuristic = false;
    cyclic_cg_preferred_operators = false;
    makespan_heuristic = false;
    makespan_heuristic_preferred_operators = false;
    no_heuristic = false;

    g_values = GTimestamp;
    g_weight = 0.5;

    queueManagementMode = BestFirstSearchEngine::PRIORITY_BASED;

    use_tss_known = false;

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

    if(!cyclic_cg_heuristic && !makespan_heuristic && !no_heuristic) {
        cerr << "Error: you must select at least one heuristic!" << endl
            << "If you are unsure, choose options \"yY\" / cyclic_cg_heuristic." << endl;
        ret = false;
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
    if(!lazy_evaluation) {
        cerr << "WARNING: Disabling lazy_evaluation is experimental at the moment." << endl;
    }
    if(use_tss_known) {
        cerr << "WARNING: tss known is experimental" << endl;
    }

    return ret;
}

void PlannerParameters::dump() const
{
    cout << endl << "Planner Paramters:" << endl;
    cout << "Anytime Search: " << (anytime_search ? "Enabled" : "Disabled") << endl;
    cout << "Timeout if plan was found: " << timeout_if_plan_found << " seconds";
    if(timeout_if_plan_found == 0)
        cout << " (no timeout)";
    cout << endl;
    cout << "Timeout while no plan was found: " << timeout_while_no_plan_found << " seconds";
    if(timeout_while_no_plan_found == 0)
        cout << " (no timeout)";
    cout << endl;
    cout << "Greedy Search: " << (greedy ? "Enabled" : "Disabled") << endl;
    cout << "Lazy Heuristic Evaluation: " << (lazy_evaluation ? "Enabled" : "Disabled") << endl;

    cout << "Cyclic CG heuristic: " << (cyclic_cg_heuristic ? "Enabled" : "Disabled")
        << " \tPreferred Operators: " << (cyclic_cg_preferred_operators ? "Enabled" : "Disabled") << endl;
    cout << "Makespan heuristic: " << (makespan_heuristic ? "Enabled" : "Disabled")
        << " \tPreferred Operators: " << (makespan_heuristic_preferred_operators ? "Enabled" : "Disabled") << endl;
    cout << "No Heuristic: " << (no_heuristic ? "Enabled" : "Disabled") << endl;

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
        case BestFirstSearchEngine::HIERARCHICAL:
            cout << "Hierarchical";
            break;
    }
    cout << endl;

    cout << "TSSKnown filtering: " << (use_tss_known ? "Enabled" : "Disabled") << endl;

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
    nhPriv.param("anytime_search", anytime_search, anytime_search);
    nhPriv.param("timeout_if_plan_found", timeout_if_plan_found, timeout_if_plan_found);
    nhPriv.param("timeout_while_no_plan_found", timeout_while_no_plan_found, timeout_while_no_plan_found); 

    nhPriv.param("greedy", greedy, greedy);
    nhPriv.param("lazy_evaluation", lazy_evaluation, lazy_evaluation);

    nhPriv.param("cyclic_cg_heuristic", cyclic_cg_heuristic, cyclic_cg_heuristic);
    nhPriv.param("cyclic_cg_heuristic_preferred_operators", 
            cyclic_cg_preferred_operators, cyclic_cg_preferred_operators);
    nhPriv.param("makespan_heuristic", makespan_heuristic, makespan_heuristic);
    nhPriv.param("makespan_heuristic_preferred_operators", makespan_heuristic_preferred_operators, 
            makespan_heuristic_preferred_operators);
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
        } else if(qMode == "hierarchical") {
            queueManagementMode = BestFirstSearchEngine::HIERARCHICAL;
        } else {
            ROS_FATAL("Unknown value: %s for queue management mode. Valid values: [priority_based, round_robin, hierarchical]", qMode.c_str());
            ret = false;
        }
    }

    nhPriv.param("use_tss_known", use_tss_known, use_tss_known);

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
    printf("  t <timeout secs> - timeout in seconds for anytime search (doesnt stop until plan found)\n");
    printf("  m <monitor file> - monitor plan, validate a given plan\n");
    printf("  g - perform greedy search (follow heuristic)\n");
    printf("  l - lazy evaluation (Use parent's f instead of child's)\n");
    printf("  y - cyclic cg CEA heuristic\n");
    printf("  Y - cyclic cg CEA heuristic - preferred operators\n");
    printf("  x - cyclic cg makespan heuristic \n");
    printf("  X - cyclic cg makespan heuristic - preferred operators\n");
    printf("  G [m|c|t|w] - G value evaluation, one of m - makespan, c - pathcost, t - timestamp, w [weight] - weighted / Note: One of those has to be set!\n");
    printf("  Q [r|p|h] - queue mode, one of r - round robin, p - priority, h - hierarchical\n");
    printf("  K - use tss known filtering (might crop search space\n");
    printf("  n - no_heuristic\n");
    printf("  p <plan file> - plan filename prefix\n");
    printf("  M v - monitoring: verify timestamps\n");
}

bool PlannerParameters::readCmdLineParameters(int argc, char** argv)
{
    //Check compliance with scripts
    //A: tfd_plan (ROS direct - should behave like any ros node) -- Requirements Name=file?
    //B: tfd_plan_eval -- should somehow work with ROS, ideally like A, but also be evaluatable in multiple parallel runs, maybe without core ... ?!?

    for (int i = 1; i < argc; i++) {
        /*if(strncmp(argv[i], "__", 2) == 0)      // ignore ros options here
          continue;*/
        // FIXME: ros::init removed all those -- anything left over at this point is for the planner
        for (const char *c = argv[i]; *c != 0; c++) {
            if (*c == 'a') {
                anytime_search = true;
            } else if (*c == 't') {
                assert(i + 1 < argc);
                timeout_if_plan_found = atoi(string(argv[++i]).c_str());
            } else if (*c == 'm') {
                assert(i + 1 < argc);
                planMonitorFileName = string(argv[++i]);
            } else if (*c == 'g') {
                greedy = true;
            } else if (*c == 'l') {
                lazy_evaluation = true;
            } else if (*c == 'y') {
                cyclic_cg_heuristic = true;
            } else if (*c == 'Y') {
                cyclic_cg_preferred_operators = true;
            } else if (*c == 'x') {
                makespan_heuristic = true;
            } else if (*c == 'X') {
                makespan_heuristic_preferred_operators = true;
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
                } else if (*g == 'p') {
                    queueManagementMode = BestFirstSearchEngine::PRIORITY_BASED;
                } else {
                    assert(*g == 'h');
                    queueManagementMode = BestFirstSearchEngine::HIERARCHICAL;
                }
            } else if (*c == 'n') {
                no_heuristic = true;
            } else if (*c == 'K') {
                use_tss_known = true;
            } else if (*c == 'p') {
                assert(i + 1 < argc);
                plan_name = string(argv[++i]);
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

