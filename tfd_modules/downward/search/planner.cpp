#include "best_first_search.h"
#include "cyclic_cg_heuristic.h"
#include "no_heuristic.h"
#include "greedy_apply_heuristic.h"
#include "monitoring.h"

#include "globals.h"
#include "operator.h"
#include "partial_order_lifter.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

#include <cstdio>
#include <math.h>
#include "plannerParameters.h"
#include "analysis.h"
#include "ros_printouts.h"

#if ROS_BUILD
#include <ros/ros.h>
#endif

#include "pddlModuleLoaderLDL.h"
#include "tfd_modules/opl/stringutil.h"

using namespace std;

#include <sys/times.h>
#include <sys/time.h>

double save_plan(BestFirstSearchEngine& engine, double best_makespan, int &plan_number, string &plan_name);
std::string getTimesName(const string & plan_name);    ///< returns the file name of the .times file for plan_name
double getCurrentTime();            ///< returns the system time in seconds

int main(int argc, char **argv)
{
#if ROS_BUILD
    ros::init(argc, argv, "tfd_modules", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
#endif

    ifstream file("../preprocess/output");
    if(strcmp(argv[argc - 1], "-eclipserun") == 0) {
        cin.rdbuf(file.rdbuf());
        cerr.rdbuf(cout.rdbuf());
        argc--;
    }

    struct tms start, search_start, search_end;
    times(&start);
    double start_walltime, search_start_walltime, search_end_walltime;
    start_walltime = getCurrentTime();

    if(!g_parameters.readParameters(argc, argv)) {
        cerr << "Error in reading parameters.\n";
        return 2;
    }
    g_parameters.dump();

    bool poly_time_method = false;
    cin >> poly_time_method;
    if(poly_time_method) {
        cout << "Poly-time method not implemented in this branch." << endl;
        cout << "Starting normal solver." << endl;
    }

    g_module_loader = new PDDLModuleLoaderLDL();

    read_everything(cin);

    g_let_time_pass = new Operator(false);
    g_wait_operator = new Operator(true);

    // init modules
    for (vector<InitModule*>::iterator it = g_init_modules.begin(); it
            != g_init_modules.end(); it++) {
        (*it)->execInit();
    }

    // init opl callback interface
    if (g_oplinit != NULL)
    {
        g_OplModuleCallback = g_oplinit->execInit(g_objectTypes, g_pred_mapping, g_func_mapping, g_pred_constants, g_func_constants);
        g_OplModuleCallback->setCurrentState(g_initial_state);
    }

    if(g_parameters.lazy_state_module_evaluation < 0) {
        if(g_condition_modules.empty() && g_cost_modules.empty())
            g_parameters.lazy_state_module_evaluation = 0;
        else
            g_parameters.lazy_state_module_evaluation = 1;
        cout << "Auto-Set Lazy State Module Evaluation to "
            << (g_parameters.lazy_state_module_evaluation ? "Enabled" : "Disabled") << endl;
    }

    FILE* timeDebugFile = NULL;
    if(!getTimesName(g_parameters.plan_name).empty()) {
        timeDebugFile = fopen(getTimesName(g_parameters.plan_name).c_str(), "w");
        if(!timeDebugFile) {
            cout << "WARNING: Could not open time debug file at: " << getTimesName(g_parameters.plan_name) << endl;
        } else {
            fprintf(timeDebugFile, "# Makespans for created plans and the time it took to create the plan\n");
            fprintf(timeDebugFile, "# The special makespan: -1 "
                    "indicates the total runtime and not the generation of a plan\n");
            fprintf(timeDebugFile, "# makespan search_time(s) total_time(s) search_walltime(s) total_walltime(s)\n");
            fflush(timeDebugFile);
        }
    }


    // Monitoring mode
    if (!g_parameters.planMonitorFileName.empty()) {
        bool ret = MonitorEngine::validatePlan(g_parameters.planMonitorFileName);
        ROS_INFO_STREAM("Monitoring: Plan is valid: " << ret);

        // for outputting statistics, etc.
        ROS_INFO("Forcing exit modules at end of monitoring with empty plan.");
        g_setModuleCallbackState(NULL);
        modules::RawPlan empty_raw_plan;
        for(vector<ExitModule*>::iterator it = g_exit_modules.begin(); it != g_exit_modules.end(); it++) {
          (*it)->execExit(empty_raw_plan, 2042);
        }

        if(ret)
            exit(0);
        exit(1);
    }

    // Initialize search engine and heuristics
    BestFirstSearchEngine* engine = new BestFirstSearchEngine(g_parameters.queueManagementMode);

    if(g_parameters.makespan_heuristic || g_parameters.makespan_heuristic_preferred_operators)
        engine->add_heuristic(new CyclicCGHeuristic(CyclicCGHeuristic::SUFFIX_MAKESPAN),
            g_parameters.makespan_heuristic, g_parameters.makespan_heuristic_preferred_operators);
    if(g_parameters.cyclic_cg_heuristic || g_parameters.cyclic_cg_preferred_operators)
        engine->add_heuristic(new CyclicCGHeuristic(CyclicCGHeuristic::CEA), g_parameters.cyclic_cg_heuristic,
            g_parameters.cyclic_cg_preferred_operators);
    if(g_parameters.no_heuristic)
        engine->add_heuristic(new NoHeuristic, g_parameters.no_heuristic, false);

    if(g_parameters.greedy_apply_heuristic || g_parameters.greedy_apply_heuristic_preferred_operators)
        engine->add_heuristic(new GreedyApplyHeuristic(), g_parameters.greedy_apply_heuristic,
                g_parameters.greedy_apply_heuristic_preferred_operators);

    double best_makespan = REALLYBIG;
    times(&search_start);
    search_start_walltime = getCurrentTime();
    int plan_number = 1;

    SearchEngine::status search_result = SearchEngine::IN_PROGRESS;
    engine->initialize();
    while(true) {
        search_result = engine->search();

        times(&search_end);
        search_end_walltime = getCurrentTime();
        if(engine->found_solution()) {
            if(search_result == SearchEngine::SOLVED) {
                // FIXME only save_plan if return value is SOLVED, otherwise no new plan was found
                best_makespan = save_plan(*engine, best_makespan, plan_number, g_parameters.plan_name);
                // write plan length and search time to file
                if(timeDebugFile && search_result == SearchEngine::SOLVED) {    // don't write info for timeout
                    int search_ms = (search_end.tms_utime - search_start.tms_utime) * 10;
                    int total_ms = (search_end.tms_utime - start.tms_utime) * 10;
                    double search_time = 0.001 * (double)search_ms;
                    double total_time = 0.001 * (double)total_ms;
                    double search_time_wall = search_end_walltime - search_start_walltime;
                    double total_time_wall = search_end_walltime - start_walltime;

                    fprintf(timeDebugFile, "%f %f %f %f %f\n", best_makespan, search_time, total_time, 
                            search_time_wall, total_time_wall);
                    fflush(timeDebugFile);
                }
                engine->bestMakespan = best_makespan;
            }
            // to continue searching we need to be in anytime search and the ret value is SOLVED
            // all other possibilities are either a timeout or completely explored search space
            if(g_parameters.anytime_search) {
                if (search_result == SearchEngine::SOLVED) {
                    // we got a plan, so we can continue searching. The next
                    // search call will start with step() that expects the step to be set up
                    // We trigger for next step by calling fetch_next_state manually
                    search_result = engine->fetch_next_state();
                    // It might happen that right after finding the goal the search is 
                    // completed (queues empty) and fetch_next_state could not do anything.
                    // In that case we should NOT call search() again as step() will reuse
                    // the last step that was and still is set up.
                    // So, catch here, that fetch_next_state finished.
                    if(search_result != SearchEngine::IN_PROGRESS)
                        break;
                } else {
                    break;
                }
            } else {
                break;
            }
        } else {
            break;
        }
    }

    // for outputting statistics, etc.
    ROS_INFO("Forcing exit modules at end with empty plan.");
    g_setModuleCallbackState(NULL);
    modules::RawPlan empty_raw_plan;
    for(vector<ExitModule*>::iterator it = g_exit_modules.begin(); it != g_exit_modules.end(); it++) {
        (*it)->execExit(empty_raw_plan, plan_number + 1000);
    }

    double search_time_wall = search_end_walltime - search_start_walltime;
    double total_time_wall = search_end_walltime - start_walltime;

    int search_ms = (search_end.tms_utime - search_start.tms_utime) * 10;
    int total_ms = (search_end.tms_utime - start.tms_utime) * 10;
    double search_time = 0.001 * (double)search_ms;
    double total_time = 0.001 * (double)total_ms;
    cout << "Search time: " << search_time << " seconds - Walltime: "
        << search_time_wall << " seconds" << endl;
    cout << "Total time: " << total_time << " seconds - Walltime: " 
        << total_time_wall << " seconds" << endl;

    if(timeDebugFile) {
        fprintf(timeDebugFile, "%f %f %f %f %f\n", -1.0, search_time, total_time, 
                search_time_wall, total_time_wall);
        fclose(timeDebugFile);
    }

    //engine->dump_everything();

    time_t now = time(NULL);
    engine->statistics(now);

    switch(search_result) {
        case SearchEngine::SOLVED_TIMEOUT:
        case SearchEngine::FAILED_TIMEOUT:
            return 137;
        case SearchEngine::SOLVED:
        case SearchEngine::SOLVED_COMPLETE:
            return 0;
        case SearchEngine::FAILED:
            assert (!engine->found_at_least_one_solution());
            return 1;
        default:
            cerr << "Invalid Search return value: " << search_result << endl;
    }

    return 2;
}

/// Epsilonize the plan at filename
/**
 * Uses a syscall to epsilonize_plan in tfd_modules package.
 *
 * This will result in the file "filename" copied at "filename.orig" and
 * "filename" being the epsilonized version.
 *
 * \returns true, if the syscall was successfull.
 */
bool epsilonize_plan(const std::string & filename, bool keepOriginalPlan = true)
{
    // get a temp file name
    std::string orig_file = filename + ".orig";

    // first move filename to ...orig file, FIXME copy better?
    int retMove = rename(filename.c_str(), orig_file.c_str());
    if(retMove != 0) {
        cerr << __func__ << ": Error moving to orig file: " << orig_file << " from: " << filename << endl;
        return false;
    }

    // create the syscall to write the epsilonized plan
#if ROS_BUILD
    std::string syscall = "rosrun tfd_modules epsilonize_plan.py";
#else
    std::string syscall = "epsilonize_plan.py";
#endif
    // read in the orig plan filename and output to filename
    syscall += " < " + orig_file + " > " + filename;  

    // execute epsilonize
    int ret = system(syscall.c_str());
    if(ret != 0) {
        cerr << __func__ << ": Error executing epsilonize_plan as: " << syscall << endl;
        // move back instead of copying/retaining .orig although not epsilonized
        int retMove = rename(orig_file.c_str(), filename.c_str());
        if(retMove != 0) {
            cerr << __func__ << ": Error moving orig file: " << orig_file << " back to: " << filename << endl;
        }
        return false;
    }

    if(!keepOriginalPlan) {
        remove(orig_file.c_str());  // don't care about return value, can't do anything if it fails
    }

    return true;
}

void record_raw_plan(const Plan & plan, int plan_number)
{
    modules::RawPlan best_raw_plan;
    const TimeStampedState* best_raw_plan_init = NULL;

    best_raw_plan.clear();
    best_raw_plan_init = NULL;
    if(plan.empty())
        return;

    // predecessor of the first op should be init
    best_raw_plan_init = plan[0].pred;

    // plan to best_raw_plan
    for(Plan::const_iterator it = plan.begin(); it != plan.end(); it++) {
        const PlanStep & step = *it;

        std::string name = step.op->get_name();     // drive loc1 loc2
        std::vector<string> parts = StringUtil::split(name, " ");
        if(parts.empty()) {
            ROS_ERROR("%s: Bad Operator Name: %s", __func__, name.c_str());
            continue;
        }
        std::string op_name = parts[0];
        modules::ParameterList parameters;
        for(unsigned int i = 1; i < parts.size(); i++) {
            parameters.push_back(modules::Parameter("", "", parts[i]));
        }

        best_raw_plan.push_back(RawAction(op_name, parameters, step.start_time, step.duration));
    }

    // exit modules
    g_setModuleCallbackState(best_raw_plan_init);
    for (vector<ExitModule*>::iterator it = g_exit_modules.begin(); it
            != g_exit_modules.end(); it++) {
        (*it)->execExit(best_raw_plan, plan_number);
    }
}

double save_plan(BestFirstSearchEngine& engine, double best_makespan, int &plan_number, string &plan_name)
{
    const vector<PlanStep> &plan = engine.get_plan();
    const PlanTrace &path = engine.get_path();

    PartialOrderLifter partialOrderLifter(plan, path);

    Plan rescheduled_plan = plan;
    if(g_parameters.reschedule_plans)
        rescheduled_plan = partialOrderLifter.lift();

    handleSubplans(plan);

    double makespan = 0;
    for(int i = 0; i < rescheduled_plan.size(); i++) {
        double end_time = rescheduled_plan[i].start_time + rescheduled_plan[i].duration;
        makespan = max(makespan, end_time);
    }
    double original_makespan = 0;
    for (int i = 0; i < plan.size(); i++) {
        double end_time = plan[i].start_time + rescheduled_plan[i].duration;
        original_makespan = max(original_makespan, end_time);
    }

    if(g_parameters.use_subgoals_to_break_makespan_ties) {
        if(makespan > best_makespan)
            return best_makespan;

        if(time_equals(makespan, best_makespan)) {
            double sumOfSubgoals = getSumOfSubgoals(rescheduled_plan);
            if(sumOfSubgoals < engine.bestSumOfGoals) {
                cout
                    << "Found plan of equal makespan but with faster solved subgoals."
                    << endl;
                cout << "Sum of subgoals = " << sumOfSubgoals << endl;
                engine.bestSumOfGoals = sumOfSubgoals;
            } else {
                return best_makespan;
            }
        } else {
            assert(makespan < best_makespan);
            double sumOfSubgoals = getSumOfSubgoals(rescheduled_plan);
            engine.bestSumOfGoals = sumOfSubgoals;
        }
    } else {
        if(makespan >= best_makespan)
            return best_makespan;
    }

    cout << endl << "Found new plan:" << endl;
    for(int i = 0; i < plan.size(); i++) {
        const PlanStep& step = plan[i];
        printf("%.8f: (%s) [%.8f]\n", step.start_time, step.op->get_name().c_str(), step.duration);
    }
    if(g_parameters.reschedule_plans) {
        cout << "Rescheduled Plan:" << endl;
        for (int i = 0; i < rescheduled_plan.size(); i++) {
            const PlanStep& step = rescheduled_plan[i];
            printf("%.8f: (%s) [%.8f]\n", step.start_time, step.op->get_name().c_str(), step.duration);
        }
    }
    cout << "Solution with original makespan " << original_makespan
        << " found (ignoring no-moving-targets-rule)." << endl;

    record_raw_plan(rescheduled_plan, plan_number);

    // Determine filenames to write to
    FILE *file = 0;
    FILE *best_file = 0;
    std::string plan_filename;
    std::string best_plan_filename;
    if (plan_name != "-") {
        // Construct planNrStr to get 42 into "42".
        int nrLen = log10(plan_number) + 10;
        char* planNr = (char*)malloc(nrLen * sizeof(char));
        sprintf(planNr, "%d", plan_number);
        std::string planNrStr = planNr;
        free(planNr);

        // Construct filenames
        best_plan_filename = plan_name + ".best";
        if (g_parameters.anytime_search)
            plan_filename = plan_name + "." + planNrStr;
        else
            plan_filename = plan_name;

        // open files
        file = fopen(plan_filename.c_str(), "w");
        best_file = fopen(best_plan_filename.c_str(), "w");

        if(file == NULL) {
            fprintf(stderr, "%s:\n  Could not open plan file %s for writing.\n", 
                    __PRETTY_FUNCTION__, plan_filename.c_str());
            return makespan;
        }
    } else {
        file = stdout;
    }

    // Actually write the plan
    plan_number++;
    for(int i = 0; i < rescheduled_plan.size(); i++) {
        const PlanStep& step = rescheduled_plan[i];
        fprintf(file, "%.8f: (%s) [%.8f]\n", step.start_time, step.op->get_name().c_str(), step.duration);
        if (best_file) {
            fprintf(best_file, "%.8f: (%s) [%.8f]\n", 
                step.start_time, step.op->get_name().c_str(), step.duration);
        }
    }

    if (plan_name != "-") {
        fclose(file);
        if(best_file)
            fclose(best_file);
    }

    cout << "Plan length: " << rescheduled_plan.size() << " step(s)." << endl;
    if(g_parameters.reschedule_plans)
        cout << "Rescheduled Makespan   : " << makespan << endl;
    else
        cout << "Makespan   : " << makespan << endl;

    if(g_parameters.epsilonize_externally) {
        // Perform epsilonize
        if(!plan_filename.empty()) {
            bool ret_of_epsilonize_plan = epsilonize_plan(plan_filename, g_parameters.keep_original_plans);
            if(!ret_of_epsilonize_plan)
                cout << "Error while calling epsilonize plan! File: " << plan_filename << endl;
        }
        if(!best_plan_filename.empty()) {
            bool ret_of_epsilonize_best_plan = epsilonize_plan(best_plan_filename, g_parameters.keep_original_plans);
            if(!ret_of_epsilonize_best_plan)
                cout << "Error while calling epsilonize best_plan! File: " << best_plan_filename << endl;
        }
    }

    return makespan;
}

std::string getTimesName(const string & plan_name)
{
    if(plan_name.empty())
        return std::string();
    if(plan_name == "-")
        return std::string();
    return plan_name + ".times";
}

double getCurrentTime()
{
    const double USEC_PER_SEC = 1000000;

    struct timeval tv; 
    gettimeofday(&tv, 0);
    return(tv.tv_sec + (double)tv.tv_usec / USEC_PER_SEC);
}

