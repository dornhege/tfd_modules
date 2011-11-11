#include "plannerTFDM.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <sys/stat.h>
#include <ros/ros.h>
#include "planParser.h"

PlannerTFDM::PlannerTFDM(const std::string & domainFile) : PlannerInterface(domainFile)
{
   _problemFileName = "/tmp/problem.pddl";
   ROS_ASSERT(_domain.parse(domainFile));
   //_domain.dumpTree();
}

PlannerTFDM::~PlannerTFDM()
{
}

PlannerInterface::PlannerResult PlannerTFDM::plan(const SymbolicState & init, const SymbolicState & goal, Plan & plan)
{
   std::ofstream f(_problemFileName.c_str());
   if(!f.good())
      return PR_FAILURE_OTHER;

   // write problem
   f << "(define (problem p01)\n";
   f << "  (:domain " << _domain.getName() << ")\n";
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

PlannerInterface::PlannerResult PlannerTFDM::callPlanner(const std::string & domain, const std::string & problem, const std::string & planNamePrefix)
{
   const bool failOnPlannerError = false;  // default: switch off

   string curPlan = planNamePrefix + ".best";
   remove(curPlan.c_str());

   // Determine the planner call command
   std::stringstream plannerCmdStr;
   plannerCmdStr << "rosrun tfd_modules tfd_plan " << domain << " " << problem;
   plannerCmdStr << " " << planNamePrefix << " " << _timeout;
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

