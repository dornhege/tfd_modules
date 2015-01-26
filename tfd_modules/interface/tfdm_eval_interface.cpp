#include "tfd_modules/tfdm_eval_interface.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <sys/stat.h>
#include <ros/ros.h>
#include "planParser.h"
#include "domainParser.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tfd_modules, tfdm_eval_interface,
        tfd_modules::TFDMEvalInterface, continual_planning_executive::PlannerInterface)

using namespace std;

namespace tfd_modules
{
    bool setupRunLazyEvalPartialCaching()
    {
        // set params for module, etc.
        ros::NodeHandle nh("/tfd_modules");
        nh.setParam("run_name", "lazy_eval_partial_caching");
        nh.setParam("lazy_state_module_evaluation", 1);

        // record all other settings

        return true;
    }

    bool setupRunEagerEvalPartialCaching()
    {
        // set params for module, etc.
        ros::NodeHandle nh("/tfd_modules");
        nh.setParam("run_name", "eager_eval_partial_caching");
        nh.setParam("lazy_state_module_evaluation", 0);

        // record all settings

        return true;
    }


    TFDMEvalInterface::TFDMEvalInterface()
    {
        runs.insert(make_pair("lazy_eval_partial_caching", setupRunLazyEvalPartialCaching));
        runs.insert(make_pair("eager_eval_partial_caching", setupRunEagerEvalPartialCaching));
        defaultRun = "lazy_eval_partial_caching";
    }

    TFDMEvalInterface::~TFDMEvalInterface()
    {
    }

    void TFDMEvalInterface::initialize(const std::string & domainFile, const std::vector<std::string> & options)
    {
        TFDMInterface::initialize(domainFile, options);

        ROS_INFO("I am in %s", get_current_dir_name());//leak

        ros::NodeHandle nhConfig("/tfd_modules/eval");
        if(!nhConfig.getParam("eval_dir", evalOutputDir)) {
            ROS_FATAL("TFDMEvalInterface: /tfd_modules/eval/eval_dir was not set!");
            evalOutputDir = "/tmp";
        }

        // add date
        time_t rawtime;
        struct tm* timeinfo;
        char buffer[1024];

        time(&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer, 1024, "/%Y-%m-%d_%H-%M-%S", timeinfo);

        evalOutputDir += buffer;
        ROS_INFO("evalOutputDir is %s", evalOutputDir.c_str());
        nhConfig.setParam("eval_current_dir", evalOutputDir);

        int ret = mkdir(evalOutputDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if(ret != 0) {
            ROS_FATAL("Failed to create evalOutputDir: %s (%d)", evalOutputDir.c_str(), ret);
        }

        safeCopy(this->_domainFile, evalOutputDir + "/domain.pddl");

        callNumber = 0;
    }

    continual_planning_executive::PlannerInterface::PlannerResult TFDMEvalInterface::plan(
            const SymbolicState & init, const SymbolicState & goal, Plan & plan)
    {
        if(!writeProblem(init, goal))
            return PR_FAILURE_OTHER;

        if(!setupRuns()) {
            ROS_FATAL("Failed to setup runs.");
        }

        if(!recordProblemData(init)) {
            ROS_ERROR("Failed to record problem data");
        }

        PlannerResult defaultResult = PR_FAILURE_OTHER;

        for(map<string, setupRunFunction>::iterator it = runs.begin(); it != runs.end(); it++) {
            if(!setupRun(it->first)) {
                ROS_ERROR("Failed to setup run for %s", it->first.c_str());
                continue;
            }

            // call the run specific setup function
            if(! (it->second()) ) {
                ROS_ERROR("Failed run specific setup for %s", it->first.c_str());
                continue;
            }

            //if(system("rosparam dump planner_params.yaml /tfd_modules") != 0) {
            //    ROS_ERROR("Failed to dump planner_params");
            //}

            // perfrom the planner call
            std::string planNamePrefix = "plan";
            ROS_INFO("Performing planner call for: %s", it->first.c_str());
            PlannerResult result = callPlanner(this->_domainFile, _problemFileName, planNamePrefix);

            //if(system("rosparam dump planner_params_after_run.yaml /tfd_modules") != 0) {
            //    ROS_ERROR("Failed to dump planner_params after run");
            //}

            if(it->first == defaultRun) {   // actually use this one for plan + result
                std::string planName = planNamePrefix + ".best";

                defaultResult = result;

                // parse plan
                if(result == PR_SUCCESS_TIMEOUT || result == PR_SUCCESS) {
                    std::ifstream planFile(planName.c_str());
                    bool ok = PlanParser::parsePlan(planFile, plan);
                    planFile.close();
                    if(!ok) {
                        ROS_WARN_STREAM("No plan generated or failure in parsing!" << std::endl);
                        defaultResult = PR_FAILURE_OTHER;
                    }
                }
            }
        }

        // change out of the nicely set directories to prevent something (like monitoring) over-writing 
        // data in the current run dir.
        if(chdir("/tmp") != 0) {
            ROS_WARN("%s: changing to /tmp failed.", __func__);
        }

        return defaultResult;
    }

    continual_planning_executive::PlannerInterface::PlannerResult TFDMEvalInterface::callPlanner(const std::string & domain, const std::string & problem,
            const std::string & planNamePrefix)
    {
        const bool failOnPlannerError = false;  // default: switch off

        string curPlan = planNamePrefix + ".best";
        remove(curPlan.c_str());

        // set plan name
        ros::param::set("tfd_modules/plan_name", planNamePrefix);
        // Determine the planner call command
        std::stringstream plannerCmdStr;
        plannerCmdStr << "rosrun tfd_modules tfd_plan_log " << domain << " " << problem << " tfdsearch.out";
        plannerCmdStr << " __name:=tfd_modules";

        // Later: other features for optional debugging???
        std::string plannerCmd = plannerCmdStr.str();

        ROS_INFO_STREAM("calling planner: " << plannerCmd << std::endl);

        // ACTUAL PLANNER CALL
        int ret = system(plannerCmd.c_str());
        if(ret % 256 == 0)   // for child processed -> returning exitstatus * 256
            ret /= 256;
        else
            ret %= 256;
        ROS_INFO_STREAM("Planner returned: " << ret);
        //if(ret == 134) {
        //  ROS_INFO("BAD EXIT CODE");
        //  abort();
        //}

        bool plannerError = false;
        bool plannerTimeout = false;
        if(ret != 0) {
            ROS_WARN_STREAM("Something in the symbolic planner failed! (Signal " << ret << ")\n");
            plannerError = true;
            if(ret == 137) {
                plannerTimeout = true;
                ROS_WARN_STREAM("Planner timeout!" << std::endl);
            } else {
                ROS_WARN_STREAM("Planner failure: " << ret << std::endl);
                if(failOnPlannerError) {
                    ROS_FATAL("Aborting run\n");
                    exit(1);
                }
            }
        }

        // check if plan.best exists for return value
        struct stat st;
        bool planWritten = false;
        if(stat(curPlan.c_str(), &st) == 0) {
            planWritten = true;
        }

        if(planWritten) {
            if(plannerTimeout)
                return PR_SUCCESS_TIMEOUT;
            // FIXME: on error still return success as there is a new plan
            return PR_SUCCESS;
        } 

        if(plannerError) {
            if(plannerTimeout)
                return PR_FAILURE_TIMEOUT;
            return PR_FAILURE_OTHER;
        }

        // FIXME: when completely explored and no plan, the planner returns fine (no error)
        // so this really IS an error
        return PR_FAILURE_UNREACHABLE;

    }

    continual_planning_executive::PlannerInterface::PlannerResult TFDMEvalInterface::monitor(
            const SymbolicState & init, const SymbolicState & goal, const Plan & plan)
    {
        return TFDMInterface::monitor(init, goal, plan);
    }

    bool TFDMEvalInterface::setupRuns()
    {
        bool ok = true;

        callNumber++;
        char buf[1024];
        sprintf(buf, "/call%03d", callNumber);

        callDir = evalOutputDir + buf;

        // create change call dir.
        // set and record all params, things that are the same for all runs (problem, etc.)
        int ret = mkdir(callDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if(ret != 0) {
            ROS_FATAL("Failed to create callDir: %s (%d)", callDir.c_str(), ret);
            ok = false;
            return ok;
        }

        ret = chdir(callDir.c_str());
        if(ret != 0) {
            ROS_FATAL("Failed to change to callDir: %s (%d)", callDir.c_str(), ret);
            ok = false;
            return ok;
        }

        return ok;
    }

    bool TFDMEvalInterface::setupRun(const string & runName)
    {
        bool ok = true;

        // create/change to run dir, etc.
        int ret = chdir(callDir.c_str());
        if(ret != 0) {
            ROS_FATAL("Failed to change to callDir: %s (%d)", callDir.c_str(), ret);
            ok = false;
            return ok;
        }

        ret = mkdir(runName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if(ret != 0) {
            ROS_FATAL("Failed to create run dir: %s (%d)", runName.c_str(), ret);
            ok = false;
            return ok;
        }

        ret = chdir(runName.c_str());
        if(ret != 0) {
            ROS_FATAL("Failed to change to run dir: %s (%d)", runName.c_str(), ret);
            ok = false;
            return ok;
        }

        // this should be the same for all
        ok &= safeCopy(_problemFileName, "problem.pddl");

        // Clean all intermediate data (plan.1/.best/.times), params???, other output files from modules
        // FIXME: done by changing to new dir.

        return ok;
    }

    bool TFDMEvalInterface::recordProblemData(const SymbolicState & init)
    {
        bool ok = true;

        ok &= safeCopy(_problemFileName, "problem.pddl");

        // record how many objects there are per type
        map<string, int> numObjectsMap;
        // type -> object
        for(SymbolicState::TypedObjectConstIterator it = init.getTypedObjects().begin();
                it != init.getTypedObjects().end(); it++) {
            if(numObjectsMap.find(it->first) == numObjectsMap.end()) {
                numObjectsMap[it->first] = 1;
            } else {
                numObjectsMap[it->first]++;
            }
        }

        ofstream probData("problem.data");
        ok &= probData.good();
        for(map<string, int>::iterator it = numObjectsMap.begin(); it != numObjectsMap.end(); it++) {
            probData << it->first << " " << it->second << endl;
        }
        probData.close();

        return ok;
    }

    bool TFDMEvalInterface::safeCopy(const string & from, const string & to)
    {
        string cmd = "cp " + from + " " + to;
        int ret = system(cmd.c_str());
        if(ret != 0) {
            ROS_ERROR("%s: Error %d when calling \"%s\".", __func__, ret, cmd.c_str());
            return false;
        }
        return true;
    }

};

