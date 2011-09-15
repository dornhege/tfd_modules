#include "best_first_search.h"
#include "cyclic_cg_heuristic.h"
#include "no_heuristic.h"
#include "mre_heuristic.h"
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
#include "ros_printouts.h"

#if ROS_BUILD
#include <ros/ros.h>
#endif

#ifdef _WIN32
#include "pddlModuleLoaderDLL.h"
#else
#include <pddlModuleLoaderLDL.h>
#endif

using namespace std;

#ifndef _WIN32
#include <sys/times.h>
#include <sys/time.h>
#endif

pair<double, double> save_plan(const vector<PlanStep> &plan,
        const PlanTrace &path, pair<double, double> best_makespan,
        int &plan_number, string &plan_name);
void readPlanFromFile(const string& filename, vector<string>& plan);
bool validatePlan(vector<string>& plan);
void save_time(string &plan_name, double search_time, double time);

double getCurrentTime()
{
    const double USEC_PER_SEC = 1000000;

    struct timeval tv; 
    gettimeofday(&tv,0);
    return(tv.tv_sec + (double)tv.tv_usec / USEC_PER_SEC);
}

int main(int argc, char **argv)
{
#if ROS_BUILD
    ros::init(argc, argv, "tfd_modules", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
#endif

    ifstream file("../preprocess/output");
    if (strcmp(argv[argc - 1], "-eclipserun") == 0) {
        cin.rdbuf(file.rdbuf());
        cerr.rdbuf(cout.rdbuf());
        argc--;
    } else {
        cout.rdbuf(cerr.rdbuf());
    }

#ifndef _WIN32
    struct tms start, search_start, search_end;
    times(&start);
    double start_walltime, search_start_walltime, search_end_walltime;
    start_walltime = getCurrentTime();
#endif
    bool poly_time_method = false;

    if(!g_parameters.readParameters(argc, argv)) {
        cerr << "Error in reading parameters.\n";
        return 2;
    }
    g_parameters.dump();

    cin >> poly_time_method;
    if (poly_time_method) {
        cout << "Poly-time method not implemented in this branch." << endl;
        cout << "Starting normal solver." << endl;
        // cout << "Aborting." << endl;
        // return 1;
    }

#ifdef _WIN32
    g_module_loader = new PDDLModuleLoaderDLL();
#else
    g_module_loader = new PDDLModuleLoaderLDL();
#endif
    read_everything(cin);

    /*
       for(map<int, ConditionModule*>::iterator it =  g_condition_modules.begin(); it != g_condition_modules.end(); it++)
       it->second->dump();
       for(vector<EffectModule *>::iterator it =  g_effect_modules.begin(); it != g_effect_modules.end(); it++)
       (*it)->dump();
       */

    g_let_time_pass = new Operator(false);
    g_wait_operator = new Operator(true);

    // init modules
    for (vector<InitModule*>::iterator it = g_init_modules.begin(); it
            != g_init_modules.end(); it++) {
        (*it)->execInit();
    }

    FILE* timeDebugFile = NULL;
    if(!g_parameters.time_debug_file.empty()) {
        timeDebugFile = fopen(g_parameters.time_debug_file.c_str(), "w");
        if(!timeDebugFile) {
            cout << "WARNING: Could not open time debug file at: " << g_parameters.time_debug_file << endl;
        } else {
            fprintf(timeDebugFile, "# makespan search_time(s)\n");
            fflush(timeDebugFile);
        }
    }


    //    dump_DTGs();
    //    dump_everything();
    //
    //    RelaxedState rs = RelaxedState(*g_initial_state);
    //    buildTestState(*g_initial_state);
    //    cout << "test state:" << endl;
    //    g_initial_state->dump();
    //    rs = RelaxedState(*g_initial_state);

    if (!g_parameters.planMonitorFileName.empty()) {
        vector<string> plan;
        readPlanFromFile(g_parameters.planMonitorFileName, plan);
        bool ret = validatePlan(plan);
        ROS_INFO_STREAM("Monitoring: Plan is valid: " << ret);
        if(ret)
            exit(0);
        exit(1);
    }

    g_engine = new BestFirstSearchEngine(g_parameters.queueManagementMode);
    if (g_parameters.makespan_heuristic || g_parameters.makespan_heuristic_preferred_operators)
        g_engine->add_heuristic(new CyclicCGHeuristic(
                    CyclicCGHeuristic::SUFFIX_MAKESPAN), g_parameters.makespan_heuristic,
                g_parameters.makespan_heuristic_preferred_operators);
    if (g_parameters.cyclic_cg_heuristic || g_parameters.cyclic_cg_preferred_operators)
        g_engine->add_heuristic(new CyclicCGHeuristic(CyclicCGHeuristic::CEA),
                g_parameters.cyclic_cg_heuristic, g_parameters.cyclic_cg_preferred_operators);
    if (g_parameters.no_heuristic)
        g_engine->add_heuristic(new NoHeuristic, g_parameters.no_heuristic, false);

    pair<double, double> best_makespan = make_pair(REALLYBIG, REALLYBIG); // first: original makespan, second: recheduled makespan

    int plan_number = 1;
#ifndef _WIN32
    times(&search_start);
    search_start_walltime = getCurrentTime();
#endif
    while (true) {
        g_engine->initialize();
        //time_t now = time(NULL);
        //g_engine->statistics(now);
        SearchEngine::status is_solved = g_engine->search();
#ifndef _WIN32
        times(&search_end);
        search_end_walltime = getCurrentTime();
#endif
        if (g_engine->found_solution()) {
            best_makespan
                = save_plan(g_engine->get_plan(), g_engine->get_path(),
                        best_makespan, plan_number, g_parameters.plan_name);
            // write plan length and search time to file
            if(timeDebugFile) {
                int search_ms = (search_end.tms_utime - search_start.tms_utime) * 10;
                double search_time = 0.001 * (double)search_ms;
                fprintf(timeDebugFile, "%f %f\n", best_makespan.second, search_time);
                fflush(timeDebugFile);
            }
            g_engine->bestMakespan = best_makespan.second;
            if (g_parameters.anytime_search) {
                if (is_solved == SearchEngine::SOLVED) {
                    g_engine->fetch_next_state();
                }
            } else {
                break;
            }
        } else {
            break;
        }
        /*if(g_engine->found_solution()) {
          best_makespan = save_plan(g_engine->get_plan(),best_makespan,plan_number,g_parameters.plan_name);

          cerr << "Plan found!" << endl;
          MonitorEngine* mon = MonitorEngine::getInstance();
          bool monitor = mon->validatePlan(g_engine->get_plan());
          cerr << "Plan validated as " << ((monitor) ? "valid!" : "not valid!") << endl;
          vector<string> plan;

          for(unsigned int i = 0; i < g_engine->get_plan().size(); i++)
          {
          stringstream tmp;
          tmp << g_engine->get_plan()[i].start_time << ": " << "(" << g_engine->get_plan()[i].op->get_name() << ") [" << g_engine->get_plan()[i].duration << "]\n";
          plan.push_back(tmp.str());
          }
          monitor = mon->validatePlan(plan);
          cerr << "Plan (string) validated as " << ((monitor) ? "valid!" : "not valid!") << endl;
          break;
          }
          else {
          break;
          }*/
    }
    //   g_engine->statistics();

    if(timeDebugFile)
        fclose(timeDebugFile);

#ifndef _WIN32
    double search_time_wall = search_end_walltime - search_start_walltime;
    double total_time_wall = search_end_walltime - start_walltime;

    int search_ms = (search_end.tms_utime - search_start.tms_utime) * 10;
    cout << "Search time: " << search_ms / 1000.0 << " seconds - Walltime: "
        << search_time_wall << " seconds" << endl;
    int total_ms = (search_end.tms_utime - start.tms_utime) * 10;
    cout << "Total time: " << total_ms / 1000.0 << " seconds - Walltime: " 
        << total_time_wall << " seconds" << endl;
    save_time(g_parameters.plan_name, search_ms, total_ms);
#endif

    return g_engine->found_at_least_one_solution() ? 0 : 1;
}

void readPlanFromFile(const string& filename, vector<string>& plan)
{
    ifstream fin(filename.c_str(), ifstream::in);
    string buffer;
    while (fin.good()) {
        getline(fin, buffer, '\n');
        if(!buffer.empty())
            plan.push_back(buffer);
    }
    fin.close();
}

bool validatePlan(vector<string> & plan)
{
    MonitorEngine* mon = MonitorEngine::getInstance();
    bool monitor = mon->validatePlan(plan);
    return monitor;
}

void save_time(string&plan_name, double search_time, double time)
{
    if (plan_name == "-") {
        return;
    }
    int len = static_cast<int> (plan_name.size() + 20);
    char *time_filename = (char*) malloc(len * sizeof(char));
    if (g_parameters.anytime_search)
        sprintf(time_filename, "%s.total.time", plan_name.c_str());
    else
        sprintf(time_filename, "%s.time", plan_name.c_str());
    FILE *file = fopen(time_filename, "w");
    if(file == NULL) {
        fprintf(stderr, "%s:\n  Could not open file %s.\n", __PRETTY_FUNCTION__, time_filename);
        free(time_filename);
        return;
    }
    fprintf(file, "%.8f %.8f\n", search_time, time);
    fclose(file);
    free(time_filename);
}

double getSumOfSubgoals(const vector<PlanStep> &plan)
{
    double ret = 0.0;
    for (int i = 0; i < plan.size(); ++i) {
        ret += plan[i].duration;
    }
    return ret;
}

double getSumOfSubgoals(const PlanTrace &path)
{
    double ret = 0.0;
    for (int i = 0; i < g_goal.size(); ++i) {
        assert(g_variable_types[g_goal[i].first] == logical || g_variable_types[g_goal[i].first] == comparison);
        //        cout << "Goal " << i << ": " << g_variable_name[g_goal[i].first] << " has to be " << g_goal[i].second << endl;
        double actualIncrement = 0.0;
        for (int j = path.size() - 1; j >= 0; --j) {
            //            cout << "At timstamp " << path[j]->timestamp << " it is " << path[j]->state[g_goal[i].first] << endl;
            if (double_equals(path[j]->state[g_goal[i].first], g_goal[i].second)) {
                actualIncrement = path[j]->timestamp;
            } else {
                //                cout << "Goal " << i << " is satiesfied at timestamp " << actualIncrement << endl;
                break;
            }
        }
        ret += actualIncrement;
    }
    return ret;
}

/// Epsilonize the plan at filename
/**
 * Uses a syscall to epsilonize_plan in tfd_modules package.
 *
 * \returns true, if the syscall was successfull.
 */
bool epsilonize_plan(const std::string & filename)
{
    // get a temp file name
    char* orig_file = new char[filename.length() + 10];
    sprintf(orig_file, "%s.orig", filename.c_str());

    // first move filename to ...orig file
    int retMove = rename(filename.c_str(), orig_file);
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
    syscall += " < " + std::string(orig_file) + " > " + filename;  

    // execute
    int ret = system(syscall.c_str());
    if(ret != 0) {
        cerr << __func__ << ": Error executing epsilonize_plan as: " << syscall << endl;
        delete orig_file;
        return false;
    }

    return true;
}

pair<double, double> save_plan(const vector<PlanStep> &plan,
        const PlanTrace &path, pair<double, double> best_makespan,
        int &plan_number, string &plan_name)
{
    /*for(int i = 0; i < path.size(); ++i) {
      path[i]->dump();
      }*/
    const bool RESCHEDULE = false;

    PartialOrderLifter partialOrderLifter(plan, path);

    /*    for(int i = 0; i < plan.size(); ++i) {
          plan[i].dump();
          }

          cout << "-------------------------------------" << endl;
          for(int i = 0; i < path.size(); ++i) {
          cout << "PATH STEP: " << i << endl;
          path[i]->dump();
          cout << "-------------------------------------" << endl;
          }
          */

    Plan new_plan = plan;
    if (RESCHEDULE)
        new_plan = partialOrderLifter.lift();

    // Handle subplans
    for (vector<SubplanModuleSet>::iterator it = g_subplan_modules.begin(); it
            != g_subplan_modules.end(); it++) {
        SubplanModuleSet sm = *it;
        Module* generateSp = std::tr1::get<0>(sm);
        Module* outputSp = std::tr1::get<1>(sm);
        Module* execSubplan = std::tr1::get<2>(sm);

        subplanGeneratorType genFn = g_module_loader->getSubplanGenerator(
                generateSp->libCall);
        outputSubplanType outputFn = g_module_loader->getOutputSubplan(
                outputSp->libCall);
        executeModulePlanType execFn = g_module_loader->getExecuteModulePlan(
                execSubplan->libCall);
        if (genFn == NULL || outputFn == NULL || execFn == NULL) {
            cerr << "Error in loading subplan generators." << endl;
            continue;
        }

        // make this temporal and another, read: better interface for outputting plans
        modulePlanType subplan;
        stringstream ss;
        for (int i = 0; i < plan.size(); i++) {
            const PlanStep& step = plan[i];
            ParameterList pl;
            // Just pass bogus as this should have been cached! warn somehow, pass "warn param"?
            subplanType spt = genFn("blubb", pl, NULL, NULL, 0, new std::pair<
                    const TimeStampedState*, const Operator*>(step.pred,
                        step.op), compareContext);
            ss << outputFn(spt) << endl << endl;
            subplan.push_back(spt);
        }
        cout << "SubplanPlan:" << endl;
        cout << ss.str() << endl;
        execFn(subplan);
    }

    double makespan = 0;
    for (int i = 0; i < new_plan.size(); i++) {
        double end_time = new_plan[i].start_time + new_plan[i].duration;
        makespan = max(makespan, end_time);
    }

    double original_makespan = 0;
    for (int i = 0; i < plan.size(); i++) {
        double end_time = plan[i].start_time + new_plan[i].duration;
        original_makespan = max(original_makespan, end_time);
    }

    //    double sumOfSubgoals = getSumOfSubgoals(path);
    double sumOfSubgoals = getSumOfSubgoals(new_plan);

    //free PlanTrace!
    for (int i = 0; i < path.size(); ++i) {
        delete (path[i]);
    }

    if (makespan > best_makespan.second)
        return best_makespan;
    if (double_equals(makespan, best_makespan.second)) {
        //        cout << "Sum of goals: " << sumOfSubgoals << endl;
        if (sumOfSubgoals < g_engine->bestSumOfGoals) {
            cout
                << "Found plan of equal makespan but with faster solved subgoals."
                << endl;
            cout << "Sum of subgoals = " << sumOfSubgoals << endl;
            g_engine->bestSumOfGoals = sumOfSubgoals;
        } else {
            return best_makespan;
        }
    } else {
        assert(makespan < best_makespan.second);
        g_engine->bestSumOfGoals = sumOfSubgoals;
    }

    cout << "Plan:" << endl;
    for (int i = 0; i < plan.size(); i++) {
        const PlanStep& step = plan[i];
        fprintf(stderr, "%.8f: (%s) [%.8f]\n", step.start_time,
                step.op->get_name().c_str(), step.duration);
    }

    cout << "new Plan:" << endl;
    for (int i = 0; i < new_plan.size(); i++) {
        const PlanStep& step = new_plan[i];
        fprintf(stderr, "%.8f: (%s) [%.8f]\n", step.start_time,
                step.op->get_name().c_str(), step.duration);
    }
    cout << "Solution with original makespan " << original_makespan
        << " found (ignoring no-moving-targets-rule)." << endl;
    if (RESCHEDULE)
        cout << "Solution was epsilonized and rescheduled to a makespan of "
            << makespan << "." << endl;
    else
        cout << "Solution was epsilonized to a makespan of " << makespan << "."
            << endl;

    FILE *file = 0;
    FILE *best_file = 0;
    char* plan_filename = NULL;
    char* best_plan_filename = NULL;
    if (plan_name != "-") {
        int len = static_cast<int> (plan_name.size() + 5 + log10(plan_number) + 10);
        best_plan_filename = (char*) malloc(len * sizeof(char));
        sprintf(best_plan_filename, "%s.best", plan_name.c_str());
        plan_filename = (char*) malloc(len * sizeof(char));
        if (g_parameters.anytime_search)
            sprintf(plan_filename, "%s.%d", plan_name.c_str(), plan_number);
        else
            sprintf(plan_filename, "%s", plan_name.c_str());

        //printf("FILENAME: %s\n", plan_filename);
        file = fopen(plan_filename, "w");
        best_file = fopen(best_plan_filename, "w");

        if(file == NULL) {
            fprintf(stderr, "%s:\n  Could not open plan file %s.\n", __PRETTY_FUNCTION__, plan_filename);
            return make_pair(original_makespan, makespan);
        }
    } else {
        file = stdout;
    }

    plan_number++;
    for (int i = 0; i < new_plan.size(); i++) {
        const PlanStep& step = new_plan[i];
        fprintf(file, "%.8f: (%s) [%.8f]\n", step.start_time,
                step.op->get_name().c_str(), step.duration);
        if (best_file) {
            fprintf(best_file, "%.8f: (%s) [%.8f]\n", step.start_time,
                    step.op->get_name().c_str(), step.duration);
        }
        //	printf("%.8f: (%s) [%.8f]\n", step.start_time, step.op->get_name().c_str(), step.duration);
    }

    if (plan_name != "-") {
        fclose(file);
        fclose(best_file);
    }

    cout << "Plan length: " << new_plan.size() << " step(s)." << endl;
    cout << "Makespan   : " << makespan << endl;

    if(plan_filename != NULL) {
        bool ret_of_epsilonize_plan = epsilonize_plan(plan_filename);
        if(!ret_of_epsilonize_plan)
            cout << "Error while calling epsilonize plan! File: " << plan_filename << endl;
    }
    if(best_plan_filename != NULL) {
        bool ret_of_epsilonize_best_plan = epsilonize_plan(best_plan_filename);
        if(!ret_of_epsilonize_best_plan)
            cout << "Error while calling epsilonize best_plan! File: " << best_plan_filename << endl;
    }

    free(plan_filename);
    free(best_plan_filename);

    return make_pair(original_makespan, makespan);
}

