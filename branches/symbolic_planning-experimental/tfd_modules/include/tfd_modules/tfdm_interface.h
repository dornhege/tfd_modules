#ifndef TFDM_INTERFACE_H
#define TFDM_INTERFACE_H

#include <string>
#include "continual_planning_executive/plannerInterface.h"

namespace tfd_modules
{

    class TFDMInterface : public continual_planning_executive::PlannerInterface
    {
        public:
            TFDMInterface();
            ~TFDMInterface();

            /**
             * \param [in] options there should be only one entry: the moduleoptions
             */
            virtual void initialize(const std::string & domainFile, const std::vector<std::string> & options);

            virtual PlannerResult plan(const SymbolicState & init, const SymbolicState & goal, Plan & plan);

            /// Use this name for writing problem files.
            void setProblemFileName(const std::string & file) {
                _problemFileName = file;
            }

            /// Set the module options.
            /**
             * Set the entries for a module option call, e.g.
             * (loadCosts@myModules 1 file.dat)
             */
            void setModuleOptions(const std::string & options) {
                _moduleOptions = options;
            }

        protected:
            PlannerResult callPlanner(const std::string & domain, const std::string & problem, const std::string & planNamePrefix);

        protected:
            std::string _domainFile;
            std::string _domainName;
            std::string _problemFileName;
            std::string _moduleOptions;
    };

};

#endif

