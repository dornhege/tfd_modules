#ifndef PDDL_MODULE_LOADER_D_L_L_H
#define PDDL_MODULE_LOADER_D_L_L_H

#include "pddlModuleLoader.h"
#include "moduleLoaderDLL.h"

class PDDLModuleLoaderDLL : public ModuleLoaderDLL, public PDDLModuleLoader
{
   public:
      PDDLModuleLoaderDLL();
      ~PDDLModuleLoaderDLL();

};

#endif

