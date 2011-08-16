#include "pddlModuleLoader.h"

PDDLModuleLoader::PDDLModuleLoader()
{

}

PDDLModuleLoader::~PDDLModuleLoader()
{

}

modules::conditionCheckerType PDDLModuleLoader::getConditionChecker(string fnString)
{
   void* fn = getFunction(fnString);
   if(fn == NULL)
      return NULL;
   modules::conditionCheckerType ret;
   *(void **) (&ret) = fn;                   // see man page of dlsym, workaround for forbidden cast between function and object pointer
   return ret;
   //return (modules::conditionCheckerType)fn;
}

modules::applyEffectType PDDLModuleLoader::getApplyEffect(string fnString)
{
   void* fn = getFunction(fnString);
   if(fn == NULL)
      return NULL;
   modules::applyEffectType ret;
   *(void **) (&ret) = fn;
   return ret;
   //return (modules::applyEffectType)fn;
}

modules::conditionCheckerType PDDLModuleLoader::getCostChecker(string fnString)
{
   void* fn = getFunction(fnString);
   if(fn == NULL)
      return NULL;
   modules::conditionCheckerType ret;
   *(void **) (&ret) = fn;
   return ret;
   //return (modules::conditionCheckerType)fn;
}

modules::moduleInitType PDDLModuleLoader::getModuleInit(string fnString)
{
   void* fn = getFunction(fnString);
   if(fn == NULL)
      return NULL;
   modules::moduleInitType ret;
   *(void **) (&ret) = fn;
   return ret;
   //return (modules::moduleInitType)fn;
}

modules::subplanGeneratorType PDDLModuleLoader::getSubplanGenerator(string fnString)
{
   void* fn = getFunction(fnString);
   if(fn == NULL)
      return NULL;
   modules::subplanGeneratorType ret;
   *(void **) (&ret) = fn;
   return ret;
   //return (modules::subplanGeneratorType)fn;
}

modules::outputSubplanType PDDLModuleLoader::getOutputSubplan(string fnString)
{
   void* fn = getFunction(fnString);
   if(fn == NULL)
      return NULL;
   modules::outputSubplanType ret;
   *(void **) (&ret) = fn;
   return ret;
   //return (modules::outputSubplanType)fn;
}

modules::executeModulePlanType PDDLModuleLoader::getExecuteModulePlan(string fnString)
{
   void* fn = getFunction(fnString);
   if(fn == NULL)
      return NULL;
   modules::executeModulePlanType ret;
   *(void **) (&ret) = fn;
   return ret;
   //return (modules::executeModulePlanType)fn;
}

