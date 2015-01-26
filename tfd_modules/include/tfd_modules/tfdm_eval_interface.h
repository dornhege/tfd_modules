#ifndef TFDM_EVAL_INTERFACE_H
#define TFDM_EVAL_INTERFACE_H

#include <string>
#include <continual_planning_executive/plannerInterface.h>
#include "tfd_modules/tfdm_interface.h"
#include <map>

namespace tfd_modules
{

    class TFDMEvalInterface : public TFDMInterface
    {
        public:
            typedef bool (*setupRunFunction)();

            TFDMEvalInterface();
            ~TFDMEvalInterface();

            /**
             * \param [in] options a list of module inits
             */
            virtual void initialize(const std::string & domainFile, const std::vector<std::string> & options);

            virtual PlannerResult plan(const SymbolicState & init, const SymbolicState & goal, Plan & plan);

            virtual PlannerResult monitor(const SymbolicState & init, const SymbolicState & goal, const Plan & plan);
 
        protected:
            virtual PlannerResult callPlanner(const std::string & domain, const std::string & problem,
                   const std::string & planNamePrefix);

            bool setupRuns();
            bool setupRun(const string & runName);
            bool recordProblemData(const SymbolicState & init);

            /// safe as in: will spit out errors on failure
            bool safeCopy(const string & from, const string & to);

        protected:
            int callNumber;
            string evalOutputDir;

            string callDir;

            std::map<string, setupRunFunction> runs;
            string defaultRun;  ///< the one that determines the plan to be exec'd
    };

};

#endif

