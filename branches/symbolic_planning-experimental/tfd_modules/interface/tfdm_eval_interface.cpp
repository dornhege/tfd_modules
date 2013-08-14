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
        // record those params and all settings

        // nothing to set for now...
        return true;
    }

    bool setupRunEagerEvalPartialCaching()
    {
        // set params for module, etc.
        // record those params and all settings

        // nothing to set for now...
        return true;
    }


    TFDMEvalInterface::TFDMEvalInterface()
    {
        runs.insert(make_pair("lazy_eval_partial_caching", setupRunLazyEvalPartialCaching));
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
            ROS_FATAL("TFDMEvalInterface: /tfd_modules/eval/evalDir was not set!");
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

        PlannerResult defaultResult = PR_FAILURE_OTHER;

        for(map<string, setupRunFunction>::iterator it = runs.begin(); it != runs.end(); it++) {
            if(!setupRun(it->first)) {
                ROS_ERROR("Failed to setup run for %s", it->first.c_str());
                continue;
            }

            if(! (it->second()) ) {
                ROS_ERROR("Failed run specific setup for %s", it->first.c_str());
                continue;
            }

            if(system("rosparam dump planner_params.yaml /tfd_modules") != 0) {
                ROS_ERROR("Failed to dump planner_params");
            }

            // perfrom the planner call
            std::string planNamePrefix = "plan";
            ROS_INFO("Performing planner call for: %s", it->first.c_str());
            PlannerResult result = callPlanner(this->_domainFile, _problemFileName, planNamePrefix);
            // TODO tee output.

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

            if(!recordRun()) {
                ROS_ERROR("Failed to record run data");
            }
        }

        return defaultResult;
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

        safeCopy(_problemFileName, "problem.pddl");

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

        // Clean all intermediate data (plan.1/.best/.times), params???, other output files from modules
        // FIXME: done by changing to new dir.

        return ok;
    }

    bool TFDMEvalInterface::recordRun()
    {
        bool ok = true;
        // save all relevant debug data for the run + overall stuff
        return ok;
    }

    void TFDMEvalInterface::safeCopy(const string & from, const string & to)
    {
        string cmd = "cp " + from + " " + to;
        int ret = system(cmd.c_str());
        if(ret != 0) {
            ROS_ERROR("%s: Error %d when calling \"%s\".", __func__, ret, cmd.c_str());
        }
    }

};

