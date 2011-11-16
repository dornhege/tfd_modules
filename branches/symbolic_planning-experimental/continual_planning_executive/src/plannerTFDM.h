#ifndef PLANNER_T_F_D_M_H
#define PLANNER_T_F_D_M_H

#include <string>
#include "plannerInterface.h"
#include "domainParser.h"

class PlannerTFDM : public PlannerInterface
{
   public:
      PlannerTFDM(const std::string & domainFile);
      ~PlannerTFDM();

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

      std::string _problemFileName;
      std::string _moduleOptions;

      DomainParser _domain;
};

#endif

