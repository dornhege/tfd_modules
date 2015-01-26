#ifndef TFDM_INTERFACE_H
#define TFDM_INTERFACE_H

#include <string>
#include <continual_planning_executive/plannerInterface.h>

namespace tfd_modules
{

    class TFDMInterface : public continual_planning_executive::PlannerInterface
    {
        public:
            TFDMInterface();
            ~TFDMInterface();

            /**
             * \param [in] options a list of module inits
             */
            virtual void initialize(const std::string & domainFile, const std::vector<std::string> & options);

            virtual PlannerResult plan(const SymbolicState & init, const SymbolicState & goal, Plan & plan);
            
            virtual PlannerResult monitor(const SymbolicState & init, const SymbolicState & goal, const Plan & plan);
 
            virtual void setTimeout(double secs);


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
            void setModuleExitOptions(const std::string & options) {
                _moduleExitOptions = options;
            }

        protected:
            virtual bool writeProblem(const SymbolicState & init, const SymbolicState & goal) const;

            virtual PlannerResult callPlanner(const std::string & domain, const std::string & problem,
                   const std::string & planNamePrefix);
            virtual PlannerResult callMonitoring(const std::string & domain, const std::string & problem,
                   const std::string & planNamePrefix);

        protected:
            std::string _domainFile;
            std::string _domainName;
            std::string _problemFileName;
            std::string _moduleOptions;
            std::string _moduleExitOptions;
    };

};

#endif

