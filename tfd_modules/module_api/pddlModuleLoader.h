#ifndef P_D_D_L_MODULE_LOADER_H
#define P_D_D_L_MODULE_LOADER_H

#include "moduleLoader.h"
#include "tfd_modules/module_api/pddlModuleTypes.h"
#include "tfd_modules/module_api/OplCallbackInterface.h"

class PDDLModuleLoader : virtual public ModuleLoader
{
   public:
      PDDLModuleLoader();
      ~PDDLModuleLoader();

   public:
      modules::conditionCheckerType getConditionChecker(string fnString);
      modules::applyEffectType getApplyEffect(string fnString);
      modules::conditionCheckerType getCostChecker(string fnString);
      modules::groundingModuleType getGroundingModule(string fnString);

      modules::moduleInitType getModuleInit(string fnString);
      modules::moduleExitType getModuleExit(string fnString);
      modules::subplanGeneratorType getSubplanGenerator(string fnString);
      modules::outputSubplanType getOutputSubplan(string fnString);
      modules::executeModulePlanType getExecuteModulePlan(string fnString);

      modules::oplCallbackInitType oplCalbackInit(string fnString);
};

#endif

