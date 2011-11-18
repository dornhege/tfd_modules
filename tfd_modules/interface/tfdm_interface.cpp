#include "tfd_modules/tfdm_interface.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <sys/stat.h>
#include <ros/ros.h>
#include "planParser.h"
#include "domainParser.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(tfd_modules, tfdm_interface,
        tfd_modules::TFDMInterface, continual_planning_executive::PlannerInterface)

namespace tfd_modules
{

    TFDMInterface::TFDMInterface()
    {
        _problemFileName = "/tmp/problem.pddl";
    }

    TFDMInterface::~TFDMInterface()
    {
    }

    void TFDMInterface::setTimeout(double secs)
    {
        ros::param::set("tfd_modules/timeout_if_plan_found", secs);
        ros::param::set("tfd_modules/timeout_while_no_plan_found", secs);
    }

    void TFDMInterface::initialize(const std::string & domainFile, const std::vector<std::string> & options)
    {
        _domainFile = domainFile;

        DomainParser _domain;
        ROS_ASSERT(_domain.parse(_domainFile));

        _domainName = _domain.getName();
        //_domain.dumpTree();

        std::stringstream ss;
        for(std::vector<std::string>::const_iterator it = options.begin(); it != options.end(); it++) {
            ss << *it << " ";
        }
        setModuleOptions(ss.str());
    }

    continual_planning_executive::PlannerInterface::PlannerResult TFDMInterface::plan(
            const SymbolicState & init, const SymbolicState & goal, Plan & plan)
    {
        std::ofstream f(_problemFileName.c_str());
        if(!f.good())
            return PR_FAILURE_OTHER;

        // write problem
        f << "(define (problem p01)\n";
        f << "  (:domain " << _domainName << ")\n";
        f << "  (:moduleoptions " << _moduleOptions << ")\n";

        init.toPDDLProblem(f);
        goal.toPDDLGoal(f);
        f << ")\n";
        f.close();

        // call planner
        std::string planNamePrefix = "/tmp/plan";
        PlannerResult result = callPlanner(this->_domainFile, _problemFileName, planNamePrefix);
        std::string planName = planNamePrefix + ".best";

        // parse plan
        if(result == PR_SUCCESS_TIMEOUT || result == PR_SUCCESS) {
            std::ifstream planFile(planName.c_str());
            bool ok = PlanParser::parsePlan(planFile, plan);
            planFile.close();
            if(!ok) {
                ROS_WARN_STREAM("No plan generated or failure in parsing!" << std::endl);
                return PR_FAILURE_OTHER;
            }
        }

        // get result
        return result;
    }

    continual_planning_executive::PlannerInterface::PlannerResult TFDMInterface::callPlanner(const std::string & domain, const std::string & problem, const std::string & planNamePrefix)
    {
        const bool failOnPlannerError = false;  // default: switch off

        string curPlan = planNamePrefix + ".best";
        remove(curPlan.c_str());

        // set plan name
        ros::param::set("tfd_modules/plan_name", planNamePrefix);
        // Determine the planner call command
        std::stringstream plannerCmdStr;
        plannerCmdStr << "rosrun tfd_modules tfd_plan " << domain << " " << problem;
        plannerCmdStr << " __name:=tfd_modules";

        // Later: other features for optional debugging???
        std::string plannerCmd = plannerCmdStr.str();

        ROS_INFO_STREAM("calling planner:" << plannerCmd << std::endl);

        // ACTUAL PLANNER CALL
        int ret = system(plannerCmd.c_str());
        if(ret % 256 == 0)   // for child processed -> returning exitstatus * 256
            ret /= 256;
        else
            ret %= 256;
        ROS_INFO_STREAM("Planner returned: " << ret);

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

};

